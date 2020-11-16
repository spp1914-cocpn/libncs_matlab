classdef NcsPlant < handle
    % Wrapper class for (linear) subsystem actuator/plant in an NCS to provide a consistent
    % interface.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2020  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (SetAccess = immutable, GetAccess = protected)         
         actuator BufferingActuator;
    end
    
    properties(Access = private)
        actualInput;        
        plantMode;
        plantState;
        
        statistics;
    end
    
    properties (SetAccess = immutable, GetAccess = public)
       plant SystemModel = LinearSystemModel; % LinearPlant or NonlinearPlant usually 
    end
       
    properties (GetAccess = public, Dependent)
        dimState;
        dimInput;
    end
    
    methods
        function dim = get.dimState(this)
            dim = this.plant.dimState;
        end
        
        function dim = get.dimInput(this)
            dim = this.actuator.dimU;
        end
    end
    
    methods (Access = public)
        %% NcsPlant
        function this = NcsPlant(plant, actuator)
            % Class constructor.
            %
            % Parameters:
            %   >> plant (LinearPlant instance)
            %      The plant to be controlled.
            %
            %   >> actuator (BufferingActuator instance)
            %      The actuator in the NCS which applies inputs to the plant.
            %
            % Returns:
            %   << this (NcsSensor)
            %      A new NcsPlant instance.
            
            this.plant = plant;
            this.actuator = actuator;
        end
        
        %% init
        function init(this, initialPlantState)
           this.plantState = initialPlantState;           
           
           % initially, no input is present, so use default input provided
           % by actuator
           this.actualInput = this.actuator.defaultInput;
           % corresponding to last mode
           this.plantMode = this.actuator.controlSequenceLength + 1;
        end
        
        %% initStatisticsRecording        
        function initStatisticsRecording(this, maxPlantStepsSteps, maxActuatorSteps)
            assert(~isempty(this.plantState), ...
                'NcsPlant:InitStatisticsRecording', ...
                '** Cannot init statistics recording: Ensure that init() has been called before **');
            
            this.statistics.trueStates = nan(this.dimState, maxPlantStepsSteps + 1);            
            this.statistics.appliedInputs = nan(this.dimInput, maxPlantStepsSteps);
            
            % maxActuatorSteps is an educated guess: actual number of steps can
            % be different if controller adapts sampling rate an runtime
            % so actual number of steps can be larger or smaller
            this.statistics.trueModes = nan(1, maxActuatorSteps);
            this.statistics.numDiscardedControlSequences = nan(1, maxActuatorSteps);            

            this.statistics.trueStates(:, 1) = this.plantState;
        end
        
        %% getInitialPlantState
        function initialState = getInitialPlantState(this)
            initialState = this.statistics.trueStates(:, 1);
        end
        
        %% getPlantStatsForTimestep
        function [plantState, appliedInput] = getPlantStatsForTimestep(this, plantTimestep)
            plantState = this.statistics.trueStates(:, plantTimestep + 1);
            appliedInput = this.statistics.appliedInputs(:, plantTimestep);
        end
        
        %% getActuatorStatsForTimestep
        function [numDiscardedControlSeq, trueMode] = getActuatorStatsForTimestep(this, actuatorTimestep)
            numDiscardedControlSeq = this.statistics.numDiscardedControlSequences(actuatorTimestep);
            trueMode = this.statistics.trueModes(actuatorTimestep);
        end
        
        %% getStatistics
        function stats = getStatistics(this, numActuatorSteps)
            if isempty(this.statistics)
                % happens only in case initStatisticsRecording has not been
                % called before
                stats = [];
                return
            end
            
            stats.trueStates = this.statistics.trueStates;
            stats.appliedInputs = this.statistics.appliedInputs;
            
            if numActuatorSteps < numel(this.statistics.trueModes)
                stats.trueModes = this.statistics.trueModes(1:numActuatorSteps);
                stats.numDiscardedControlSequences = this.statistics.numDiscardedControlSequences(1:numActuatorSteps);
            else
                stats.trueModes =  this.statistics.trueModes;
                stats.numDiscardedControlSequences = this.statistics.numDiscardedControlSequences;
            end
        end
        
        %% plantStep
        function newPlantState = plantStep(this, plantTimestep)
            assert(~isempty(this.plantState), ...
                'NcsPlant:PlantStep', ...
                '** Plant has not been initialized: Ensure that init() has been called before **');
            
            % apply the input to proceed to the next time step
            this.plant.setSystemInput(this.actualInput);
            newPlantState = this.plant.simulate(this.plantState);
            
            % record the data about true input and state
            this.statistics.appliedInputs(:, plantTimestep) = this.actualInput; % this input was applied at time k (u_k)            
            this.statistics.trueStates(:, plantTimestep + 1) = newPlantState; % store x_{k+1}
            
            this.plantState = newPlantState;
        end
        
        %% actuatorStep
        function [plantMode, controllerAcks] = actuatorStep(this, controllerTimestep, caPackets, createAcks)
            % Process received packets (i.e., control sequences) from the
            % controller and feed a control input to the plant.
            %
            % Parameters:
            %   >> controllerTimestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      controller's sampling interval.
            %
            %   >> caPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing control sequences transmitted from the controller.
            %
            %   >> createAcks (Flag, i.e. logical scalar, optional)
            %      Flag to indicate whether (application layer) acknowledgment packets shall be created 
            %      for each received packet from the controller.
            %      If left out, the default value true is used, indicating
            %      that acks will be created.
            %
            % Returns:
            %   << plantMode (Positive integer)
            %      The mode of the underlying MJLS that corresponds to the
            %      applied input (i.e., the age of the buffered sequence), i.e., theta_k.
            %
            %   << controllerAcks (Column vector of DataPackets, might be empty)
            %      An ACK is created for each received controller packet.
            %      Empty matrix is returned in case none were received or
            %      <createAcks> is set to false.
            
            % get actual input u_k and theta_k
            if nargin == 3 || createAcks
                [this.actualInput, plantMode, controllerAcks] = this.actuator.step(controllerTimestep, caPackets);
            else
               [this.actualInput, plantMode] = this.actuator.step(controllerTimestep, caPackets);
               controllerAcks = [];
            end
            
            if plantMode == this.plantMode + 1 || plantMode == this.actuator.controlSequenceLength + 1
                % plant mode increased by one or indicates application of
                % default input -> none of the received sequences used
                numDiscardedSeq = numel(caPackets);
            else
                % plant mode is smaller or equal to previous one -> one
                % sequence became active
                numDiscardedSeq = numel(caPackets) - 1;
            end
            this.plantMode = plantMode;
            % record the number of discarded control packets            
            this.statistics.numDiscardedControlSequences(controllerTimestep) = numDiscardedSeq;
            this.statistics.trueModes(controllerTimestep) = this.plantMode; % the mode theta_k
        end        
        
        %% getInterpolatedPlantState
        function interpolatedState = getInterpolatedPlantState(this, plantTimestep, intervalPortion)
            % start from the given state and interpolate only until portion
            % is reached
            if intervalPortion == 0
                % nothing to do, so simply return state
                interpolatedState = this.statistics.trueStates(:, plantTimestep + 1); % this is the true plant state corresponding to the given time step
                return
            end
            if Checks.isClass(this.plant, 'InvertedPendulum')
                oldSamplingInterval = this.plant.samplingInterval;
                this.plant.samplingInterval = oldSamplingInterval * intervalPortion;
                % pick the input that was used
                this.plant.setSystemInput(this.statistics.appliedInputs(:, plantTimestep));
                interpolatedState = this.plant.simulate(this.statistics.trueStates(:, plantTimestep + 1));
                % restore properties
                %this.plant.setSystemInput(this.actualInput);
                this.plant.samplingInterval = oldSamplingInterval;
            else
                this.plant.setSystemInput(this.statistics.appliedInputs(:, plantTimestep));
                interpolatedState = this.statistics.trueStates(:, plantTimestep + 1);
            end
            % this function can be improved: reuse the noise 
        end        
        
        %% changeActuatorSequenceLength
        function changeActuatorSequenceLength(this, newSequenceLength)
            % Change the length of the control sequences to be processed by the actuator.
            % This operation is required if the employed controller adapts
            % the length of transmitted control sequences at runtime.
            %
            % Parameters:
            %   >> newSequenceLength (Positive integer)
            %      The new sequence length to be used.
            
            this.actuator.changeControlSequenceLength(newSequenceLength);
        end
        
        %% isStateAdmissible
        function isAdmissible = isStateAdmissible(this)
            % check if current true plant state is admissible
            isAdmissible = this.plant.isValidState(this.plantState);
        end
    end
    
end

