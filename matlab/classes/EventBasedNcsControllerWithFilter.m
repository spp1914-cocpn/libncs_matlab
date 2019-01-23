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
    %    Copyright (C) 2018  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (Access = private, Constant)
        defaultDeadband = 10;
    end
    
    properties (Access = public)
        % deadband (for the deadband control strategy)
        deadband = EventBasedNcsControllerWithFilter.defaultDeadband;
        eventTrigger = EventBasedControllerTriggerCriterion.Sequence;
    end
    
    properties (Access = private)
        lastSentData; % struct containing timestep, sequence, state estimate, perceived QoC, stage costs
    end
    
    methods
        function set.deadband(this, newDeadband)
            assert(Checks.isNonNegativeScalar(newDeadband), ...
                'EventBasedNcsControllerWithFilter:SetDeadband:InvalidDeadband', ...
                '** <newDeadband> must be a nonnegative scalar **');

            this.deadband = newDeadband;
        end
        
        function set.eventTrigger (this, newEventTrigger)
            assert(isscalar(newEventTrigger) && isa(newEventTrigger, 'uint8') ...
                && newEventTrigger <= EventBasedControllerTriggerCriterion.getMaxId(), ...
                'EventBasedNcsControllerWithFilter:SetEventTrigger:InvalidTrigger', ...
                '** <newEventTrigger> must be a EventBasedControllerTriggerCriterion **');
            
            this.eventTrigger = EventBasedControllerTriggerCriterion(newEventTrigger);
        end
    end
    
    methods (Access = public)
        %% EventBasedNcsControllerWithFilter
        function this = EventBasedNcsControllerWithFilter(controller, filter, plantModel, measModel, defaultInput, plantStateOrigin)
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
            %   >> plantStateOrigin (Vector, optional)
            %      The origin in plant state variables expressed in terms of
            %      the state variables used by the controller, e.g., the
            %      linearization point if the controller uses a linear
            %      approximation of the plant dynamics.
            %      This vector is required to compute the quality of control
            %      and the total control costs which are compute with
            %      respect to the controller's state variables.
            %      If left out, no offset is assumed, i.e., the zero vector.
            %
            % Returns:
            %   << this (EventBasedNcsControllerWithFilter)
            %      A new EventBasedNcsControllerWithFilter instance.
            %
            
            if nargin == 5
                plantStateOrigin = [];
            end
            this = this@NcsControllerWithFilter(controller, filter, plantModel, measModel, defaultInput, plantStateOrigin);            
            this.isEventBased = true;
        end
        
        %% step
        function [dataPacket, numUsedMeas, numDiscardedMeas, controllerState] ...
                = step(this, timestep, scPackets, acPackets, plantMode)
            % Compute a control sequence as part of a control cycle in an
            % NCS.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %     
            %   >> scPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing measurements taken and transmitted from the sensor.
            %
            %   >> acPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing ACKs returned from the actuator.
            %   
            %   >> plantMode (Nonnegative integer, might be empty)
            %      The previous true plant mode (theta_{k-1}), or the empty
            %      matrix, if not directly known to the controller.
            %
            % Returns:
            %   << dataPacket (DataPacket or empty matrix)
            %      The data packet containing new control sequence computed
            %      by the controller, with the individual inputs column-wise arranged,
            %      to be transmitted to the actuator.
            %      Empty matrix is returned in case none is to be
            %      transmitted, due to the applied deadband control
            %      strategy.
            %
            %   << numUsedMeas (Nonnegative integer)
            %      The number of measurements actually used for the
            %      computation of the control sequence.
            %
            %   << numDiscardedMeas (Nonnegative integer)
            %      The number of measurements which were discarded due to
            %      a too large delay.
            %
            %   << controllerState (Column vector, optional)
            %      The controller state (i.e., the controller's estimate of
            %      the plant state), expressed in terms of the plant state
            %      variables.
            
            % we make use of a dedicated filter to obtain the state
            % estimate
           
            % first, update the estimate, i.e., obtain x_k
            [numUsedMeas, numDiscardedMeas, previousMode] ...
                = this.updateControllerState(scPackets, acPackets, timestep, plantMode);
           
            % compute the control inputs u_k, ..., u_{k+N}, i.e, sequence U_k
            % use (previous!) true mode (theta_{k-1}) or estimated mode of augmented system
            % and use xhat_k
            inputSequence = this.computeControlInputSequence(previousMode, timestep);
            
            sendData.state = this.getControllerState(); % with respect to the plant coordinates
            sendData.sequence = inputSequence;
            sendData.timestep = timestep;
            sendData.qoc = this.getCurrentQualityOfControl(sendData.state, timestep); % QoC, perceived by the controller
            sendData.stageCosts = this.getCurrentStageCosts(sendData.state, ...
                    inputSequence(:, 1), timestep); % stage costs if first element of sequence was applied 
            % finally, create the data packet, if sequence shall be send           
            % update the buffer for the filter
            if this.checkSendControlSequence(sendData)
                dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
                                
                this.lastSentData = sendData;                
            else
                inputSequence = repmat(this.defaultInput, 1, this.controlSequenceLength);
                dataPacket = [];               
            end
            this.updateControlSequencesBuffer(inputSequence);           
            
            if nargout == 4
                controllerState = sendData.state;
            end
        end
    end
    
    methods (Access = private)
        %% checkSendControlSequence
        function sendSeq = checkSendControlSequence(this, sendData)
            sendSeq = isempty(this.lastSentData) ...
                || (sendData.timestep - this.lastSentData.timestep) > this.controlSequenceLength - 1 ...
                || this.eventTrigger.evaluateTrigger(this.lastSentData, sendData, this.deadband);            
        end
    end
end

