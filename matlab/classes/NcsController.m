classdef NcsController < handle
    % Wrapper class for controllers in an NCS to provide a consistent
    % interface for controllers that require an external filter or state
    % estimate, and those which don't.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2018  Florian Rosenthal <florian.rosenthal@kit.edu>
    %
    %                        Institute for Anthropomatics and Robotics
    %                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
    %                        Karlsruhe Institute of Technology (KIT), Germany
    %
    %                        http://isas.uka.de
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
    
    properties (SetAccess=immutable, GetAccess = public)
        controller@SequenceBasedController;
        % origin shift: useful, in case controller uses a linearized model
        % of the plant dynamics
        plantStateOrigin; % the origin of the plant coordinate system in controller coordinates
    end
    
    properties (SetAccess = immutable, GetAccess = public)
        % indicate whether controller works event-based
        isEventBased@logical = false;
    end
    
    properties (Dependent, GetAccess = public)
        controlSequenceLength;
    end
    
    methods
        function sequenceLength = get.controlSequenceLength(this)
            sequenceLength = this.controller.sequenceLength;
        end
    end
   
    methods (Access = public)
        %% NcsController
        function this = NcsController(controller, plantStateOrigin)
            % Class constructor.
            %
            % Parameters:
            %   >> controller (SequenceBasedController instance)
            %      The controller to be utilized within the corresponding
            %      NCS.
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
            %   << this (NcsController)
            %      A new NcsController instance.
            this.controller = controller;
            
            if Checks.isClass(this.controller, 'EventTriggeredInfiniteHorizonController')
                this.isEventBased = true;
            end
            if nargin == 2 && Checks.isVec(plantStateOrigin)
                this.plantStateOrigin = plantStateOrigin(:);
            end
        end
        
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
            %      Empty matrix is returned in case none is to be transmitted (e.g., when the controller is event-based).
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
            
            [measurements, measDelays] = NcsController.processScPackets(scPackets);
            
            if nargout == 4
                % retrieve state before sequence is computed
                controllerState = this.controller.getControllerPlantState(this);
                % shift controller state if required, to be expressed with
                % regards to the plant coordinates
                if ~isempty(this.plantStateOrigin)
                    controllerState = controllerState + this.plantStateOrigin;
                end
            end
            
            inputSequence = ...
                this.reshapeInputSequence(this.controller.computeControlSequence(measurements, measDelays));
            
            dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
            [numUsedMeas, numDiscardedMeas] = this.controller.getLastComputationMeasurementData();
        end
        
        %% computeCosts
        function controlCosts = computeCosts(this, plantStates, appliedInputs)
            % Compute accrued costs for the given state and input
            % trajectory according to this controller's underlying cost functional.
            %
            % Parameters:
            %   >> stateTrajectory (Matrix of dimension dimPlantState-by-n)
            %      A matrix representing a state (with respect to the plant model) trajectory, i.e., adjacent
            %      columns contain succesive plant states.
            %
            %   >> appliedInputs (Matrix of dimension dimPlantInput-by-m)
            %      A matrix representing an input trajectory, i.e., adjacent
            %      columns contain succesive control inputs.
            %
            % Returns:
            %   << costs (Nonnegative scalar)
            %      The accrued costs according to this controller's underlying cost functional.
            
            % we compute the accrued costs with repsect to the state
            % variables of the controller
            if isempty(this.plantStateOrigin)
                states = plantStates;
            else
                states = plantStates - this.plantStateOrigin;
            end
            controlCosts = this.controller.computeCosts(states, appliedInputs);
        end
        
        %% getCurrentStageCosts
        function stageCosts = getCurrentStageCosts(this, plantState, appliedInput, timestep)
            % Get the current stage costs for the given plant
            % true state.
            %
            % Parameters:
            %   >> plantState (Vector)
            %      The plant true state (with respect to the plant model) at the given time step.
            %
            %   >> appliedInput (Vector)
            %      The input applied to the plant at the given timestep.
            %
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            % Returns:
            %   << stageCosts (Nonnegative scalar)
            %      The current stage costs according to the controller's underlying cost functional.
                        
            % we compute the stage costs with respect to the state
            % variables of the controller
            if isempty(this.plantStateOrigin)
                state = plantState(:);
            else
                state = plantState(:) - this.plantStateOrigin;
            end
            
            stageCosts = this.controller.computeStageCosts(state, appliedInput, timestep);
        end
        
        %% getCurrentQualityOfControl
        function qoc = getCurrentQualityOfControl(this, plantState, timestep)
            % Get the current quality of control (QoC) for the given plant
            % true state.
            %
            % Parameters:
            %   >> plantState (Vector)
            %      The plant true state (with respect to the plant model) at the given time step.
            %
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            % Returns:
            %   << currentQoC (Nonnegative scalar)
            %      The current QoC, which is simply the norm of the current
            %      plant true state, or, in a tracking task, the norm of
            %      the deviation from the current reference output.
            
            % we compute the qoc with respect to the state
            % variables of the controller
            if isempty(this.plantStateOrigin)
                state = plantState;
            else
                state = plantState(:) - this.plantStateOrigin;
            end
            
            if Checks.isClass(this.controller, 'SequenceBasedTrackingController')
                % we track a reference
                qoc = norm(this.controller.getDeviationFromRefForState(state, timestep)); 
            else
                % we track the origin
                qoc = norm(state);
            end
        end
    end
  
    methods (Access = protected)
        %% reshapeInputSequence
        function sequence = reshapeInputSequence(this, inputSequence)
            % reshape the resulting stacked vector (arrange inputs column-wise)
            if ~isempty(inputSequence)
                sequence = reshape(inputSequence, [], this.controlSequenceLength);
            else
                sequence = [];
            end
        end       
       
    end
    
    methods (Static, Access = protected)
         
        %% createControlSequenceDataPacket
         function dataPacket = createControlSequenceDataPacket(timestep, inputSequence)
            % the control sequence is transmitted from the controller (id = 2) to the actuator (id = 1)
            dataPacket = CreateDataPacket(inputSequence, timestep, 2, 1);
         end
        
        %% processScPackets
        function [measurements, measDelays] = processScPackets(scPackets)
            measurements = [];
            measDelays = [];
            if numel(scPackets) ~= 0
                delays = cell(1, numel(scPackets));
                meas = cell(1, numel(scPackets));
                [delays{:}] = scPackets(:).packetDelay;
                [meas{:}] = scPackets(:).payload;
                measurements = cell2mat(meas);
                measDelays = cell2mat(delays);
            end
        end
    end
end

