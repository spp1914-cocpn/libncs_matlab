classdef EventBasedNcsControllerWithFilter < NcsControllerWithFilter
    % Wrapper class for sequence-based controllers that require an external
    % filter and support event-based data transmission in terms of a
    % deadband control strategy.
    %
    % Literature: 
    %  	Yun-Bo Zhao, Guo-Ping Liu, and David Rees,
    %   Packet-Based Deadband Control for Internet-Based Networked Control Systems,
    %   IEEE Transactions on Control Systems Technology, vol. 18, no. 5, pp. 1057-1067, 2010.
    %
    %   Jörg Fischer,
    %   Optimal sequence-based control of networked linear systems,
    %   Karlsruhe series on intelligent sensor-actuator-systems, Volume 15,
    %   KIT Scientific Publishing, 2015.    
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
    %
    %                        Institute for Anthropomatics and Robotics
    %                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
    %                        Karlsruhe Institute of Technology (KIT), Germany
    %
    %                        https://isas.iar.kit.edu
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    properties (Access = public, Constant)
        defaultDeadband = 10;
    end
    
    properties (Access = public)
        % deadband (for the deadband control strategy)
        deadband(1,1) double {mustBeNonnegative} = EventBasedNcsController.defaultDeadband;
        eventTrigger(1,1) EventBasedControllerTriggerCriterion ...
            = EventBasedControllerTriggerCriterion.Sequence;
    end
    
    properties (Access = private)
        lastSentData; % struct containing timestep, sequence, state estimate, perceived QoC, stage costs
    end
    
    properties (SetAccess = immutable, GetAccess = private)
        canUpdateEtaState;
        samplingInterval;
    end  
    
    methods (Access = public)
        %% EventBasedNcsControllerWithFilter
        function this = EventBasedNcsControllerWithFilter(controller, filter, plantModel, measModel, defaultInput, ...
                initialCaDelayProbs, controllerSamplingInterval, plantStateOrigin)
            % Class constructor.
            %
            % Parameters:
            %   >> controller (SequenceBasedController instance)
            %      The controller to be utilized within the corresponding
            %      NCS.
            %
            %   >> filter (DelayedMeasurementsFilter instance)
            %      The filter employed to supply the given controller with
            %      state estimates.
            %
            %   >> plantModel (SystemModel instance)
            %      The system model utilized by the filter for prediction
            %      steps, which might be specifically tailored to the
            %      filter.
            %
            %   >> measModel (LinearMeasurementModel instance)
            %      The measurement model utilized by the filter for update
            %      steps, which might be specifically tailored to the
            %      filter.
            %
            %   >> defaultInput (Vector)
            %      The default input to be employed by the actuator if its
            %      buffer runs empty.
            %
            %   >> initialCaDelayProbs (Nonnegative Vector)
            %      The vector describing the delay distribution of the
            %      CA-network initially assumed/used by the given
            %      controller and/or filter.
            %
            %   >> controllerSamplingInterval (Positive Scalar)
            %      A positive scalar denoting the fixed sampling interval (in
            %      seconds) of the controller.
            %
            %   >> plantStateOrigin (Vector, optional)
            %      The origin in plant state variables expressed in terms of
            %      the state variables used by the controller, e.g., the
            %      linearization point if the controller uses a linear
            %      approximation of the plant dynamics.
            %      This vector is required to compute the quality of control
            %      and the total control costs which are computed with
            %      respect to the controller's state variables.
            %      If left out, no offset is assumed, i.e., the zero vector.
            %
            % Returns:
            %   << this (EventBasedNcsControllerWithFilter)
            %      A new EventBasedNcsControllerWithFilter instance.
            %
            
            if nargin == 7
                plantStateOrigin = [];
            end
            this = this@NcsControllerWithFilter(controller, filter, plantModel, measModel, defaultInput, ...
                initialCaDelayProbs, plantStateOrigin);
            this.isEventBased = true;
            this.canUpdateEtaState = ismethod(this.controller, 'setEtaState');
            this.samplingInterval = controllerSamplingInterval;
        end
    end    
    
    methods (Access = protected)
        %% preDoStep
        function preDoStep(this, timestep)
            preDoStep@NcsControllerWithFilter(this, timestep);
            if this.canUpdateEtaState && (isempty(this.lastSentData) || this.lastSentData.timestep ~= timestep - 1)
                % the last computed sequence was not sent to plant, so
                % update controller's internal ete state
                this.updateControllerEtaState();
            end
        end
        
         %% postDoStep
        function dataPacket = postDoStep(this, inputSequence, controllerState, timestep)
            sendData.state = controllerState; % with respect to the plant coordinates
            sendData.sequence = inputSequence;
            sendData.timestep = timestep;            
            sendData.error = this.getCurrentControlError(timestep, timestep * this.samplingInterval); % control error, perceived by the controller
            sendData.stageCosts = this.getCurrentStageCosts(sendData.state, ...
                    inputSequence(:, 1), timestep); % stage costs if first element of sequence was applied 
            % finally, create the data packet, if sequence shall be send           
            % update the buffer for the filter
            if this.checkSendControlSequence(sendData)
                dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
                                
                this.lastSentData = sendData;
                this.updateControlSequencesBuffer(inputSequence);
            else
                this.updateControlSequencesBuffer(repmat(this.defaultInput, 1, this.controlSequenceLength));
                dataPacket = [];                
            end
            
            this.caDelayProbsChanged = false; % reset the flag (has been set in preDoStep)
        end
    end
    
    methods (Access = private)
        
        %% updateControllerEtaState
        function updateControllerEtaState(this)
            % we construct the eta state from the buffered sequences
            % to ensure that the controller has valid information
            dimInput = numel(this.defaultInput);
            dimEta = this.controlSequenceLength * (this.controlSequenceLength - 1) * dimInput / 2;
            newEta = zeros(dimEta, 1);
            % at time k, the first sequence in the buffer is U_{k-1}
            % the last sequence in the buffer is U_{k-N}, where the
            % sequence length is N+1
            idx = 1;
            for i=1:this.controlSequenceLength - 1
                % index i adresses buffered sequence U_{k-i}
                % from the old sequence we need the inputs u_{k|k-i},u_{k+1-i|k-i}, ..., u_{k+N-i|k-i}
                % for i= 1: u_{k|k-1},..., u_{k+N-1|k-1} 
                % for i=N (sequence length -2):u_{k|k-N}
                inputs = this.bufferedControlInputSequences(:, i+1:this.controlSequenceLength, i);
                newEta(idx:idx+numel(inputs)-1) = inputs(:);
                idx = idx + numel(inputs);
            end
            this.controller.setEtaState(newEta);
        end
        
        %% checkSendControlSequence
        function sendSeq = checkSendControlSequence(this, sendData)
            sendSeq = isempty(this.lastSentData) ...
                || (sendData.timestep - this.lastSentData.timestep) > this.controlSequenceLength - 1 ...
                || this.eventTrigger.evaluateTrigger(this.lastSentData, sendData, this.deadband);            
        end
    end
end

