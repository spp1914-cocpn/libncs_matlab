classdef NcsController < handle
    % Wrapper class for controllers in an NCS to provide a consistent
    % interface for controllers that require an external filter or state
    % estimate, and those which don't.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2020  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (Constant, Access = public)
        defaultControlErrorWindowSize = 10; % in time steps
    end
    
    properties (Access = public)
        controlErrorWindowSize(1,1) double {mustBePositive, mustBeInteger} = NcsController.defaultControlErrorWindowSize; % in time steps
    end
    
    properties (SetAccess=immutable, GetAccess = public)
        controller;
        % origin shift: useful, in case controller uses a linearized model
        % of the plant dynamics
        plantStateOrigin; % column vector, the origin of the plant coordinate system in controller coordinates
    end
    
    properties (SetAccess = immutable, GetAccess=private)
        doStepFun;
        doComputeControlErrorFun;
        canChangeCaProbsFun;
    end
            
    properties (SetAccess = immutable, GetAccess = protected)
        defaultInput; % the default input, stored as column vector
    end    
    
    properties (SetAccess = protected, GetAccess = public)       
        % indicate whether controller works event-based
        isEventBased(1,1) logical = false;  
    end
    
    properties (Access = private)
        statistics;
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
        function this = NcsController(controller, defaultInput, plantStateOrigin)
            % Class constructor.
            %
            % Parameters:
            %   >> controller (SequenceBasedController instance)
            %      The controller to be utilized within the corresponding
            %      NCS.
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
            %      and the total control costs which are computed with
            %      respect to the controller's state variables.
            %      If left out, no offset is assumed, i.e., the zero vector.
            %
            % Returns:
            %   << this (NcsController)
            %      A new NcsController instance.
            
            arguments
                controller(1,1) SequenceBasedController
                defaultInput(:, 1) double {mustBeFinite}
                plantStateOrigin(:, 1) double {mustBeFinite} = zeros(0,1)
            end

            this.controller = controller;
            this.defaultInput = defaultInput(:);
            this.plantStateOrigin = plantStateOrigin(:);
            
            if Checks.isClass(this.controller, 'EventTriggeredInfiniteHorizonController')
                this.isEventBased = true;
            end
            
            this.doStepFun = this.constructDoStepFun();
            this.doComputeControlErrorFun = this.constructComputeControlErrorFun();
            this.canChangeCaProbsFun = this.constructCanChangeCaDelayProbsFun();
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
            
            % we compute the accrued costs with respect to the state
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
            % Get the current stage costs for the given plant true state.
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
        
        %% getCurrentControlError
        function [estimatedControlError, actualControlError] ...
                = getCurrentControlError(this, timestep, plantStateHistory)
            % Get the current control error (as perceived by the
            % controller) at the given time step, and, optionally, the
            % current true error based on the given true state trajectory.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            %   >> plantStateHistory (Matrix, optional)
            %      The plant true states (with respect to the plant model),
            %      with the last column being the plant true state at the given time step.
            %
            % Returns:
            %   << estimatedControlError (Nonnegative scalar)
            %      The current estimated control error, which is an integral measure: the accumulated norm of the 
            %      controller states, or, in a tracking task,
            %      the accumulated norm of the deviation from the reference
            %      output or setpoint.
            %
            %   << actualControlError (Nonnegative scalar, optional)
            %      The current true control error, which is an integral measure: the accumulated norm of the 
            %      provided plant true states (with respect to the controller plant model), or, in a tracking task, the norm of
            %      the accumulated norm of the deviation from the reference
            %      output or setpoint.
            
            % we compute the conrol error with respect to the state
            % variables of the controller            
            endIdx = timestep + 1;
            beginIdx = max(2, endIdx - this.controlErrorWindowSize + 1);

            controllerStateHistory = this.statistics.controllerStates(:, beginIdx:endIdx);
            if isempty(this.plantStateOrigin)
                controllerStates = controllerStateHistory;
            else                
                controllerStates = controllerStateHistory - this.plantStateOrigin;
            end
            estimatedControlError = this.computeControlError(controllerStates, timestep);
            
            if nargin == 3 && nargout == 2
                if isempty(this.plantStateOrigin)
                    states = plantStateHistory(:, beginIdx:endIdx);                    
                else
                    states = plantStateHistory(:, beginIdx:endIdx) - this.plantStateOrigin;                    
                end
                actualControlError = this.computeControlError(states, timestep);
            end            
        end        
    end
  
    methods (Access = public, Sealed)
        
        %% step
        function dataPacket = step(this, timestep, scPackets, acPackets, plantMode)
            % Template method to compute a control sequence as part of a control cycle in an
            % NCS.
            % The following functions, that can be overriden by subclasses, are called:
            % 1. preDoStep();
            % 2. doStep();
            % 3. postDoStep();
            % 4. recordStatistics();
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
            
            this.preDoStep(timestep);
            [controllerState, inputSequence, numUsedMeas, numDiscardedMeas] ...
                = this.doStep(timestep, scPackets, acPackets, plantMode);
            dataPacket = this.postDoStep(inputSequence, controllerState, timestep);
            
            % update the recorded data accordingly            
            this.recordStatistics(timestep, numUsedMeas, numDiscardedMeas, controllerState);
        end
        
        %% initStatisticsRecording
        function initStatisticsRecording(this, maxLoopSteps, dimState)
            % maxLoopSteps is an educated guess: actual number of steps can
            % be different if controller adapts sampling rate an runtime
            this.statistics.numUsedMeasurements = nan(1, maxLoopSteps);
            this.statistics.numDiscardedMeasurements = nan(1, maxLoopSteps);            
            %
            this.statistics.controllerStates = zeros(dimState, maxLoopSteps + 1);
        end
        
        %% getStatistics
        function controllerStats = getStatistics(this, numControllerSteps)
            % Get the statistical data that has been recorded during a
            % simulation run of a given number of time steps.
            %
            % Parameters:
            %   >> numControllerSteps (Positive integer)
            %      The number of invocations of the controller during the
            %      simulation of the NCS.
            %
            % Returns:
            %   << statistics (Struct)
            %      The statistical data collected during the simulation.
            %
            
            if numControllerSteps < numel(this.statistics.numUsedMeasurements)
                controllerStats.numUsedMeasurements = this.statistics.numUsedMeasurements(1:numControllerSteps);
                controllerStats.numDiscardedMeasurements = this.statistics.numDiscardedMeasurements(1:numControllerSteps);
                controllerStats.controllerStates = this.statistics.controllerStates(:, 1:numControllerSteps + 1);
            elseif numControllerSteps == size(this.statistics.controllerStates, 2)
                controllerStats.numUsedMeasurements = this.statistics.numUsedMeasurements(1:numControllerSteps);
                controllerStats.numDiscardedMeasurements = this.statistics.numDiscardedMeasurements(1:numControllerSteps);
                controllerStats.controllerStates = this.statistics.controllerStates;    
            else
                controllerStats.numUsedMeasurements = this.statistics.numUsedMeasurements;
                controllerStats.numDiscardedMeasurements = this.statistics.numDiscardedMeasurements;
                controllerStats.controllerStates = this.statistics.controllerStates;               
            end
        end
        
        %% getStatisticsForTimestep
        function [numUsedMeasurements, numDiscardedMeasurements, controllerState] = getStatisticsForTimestep(this, timestep)
            % get the data receorded at the specified time step
            numUsedMeasurements = this.statistics.numUsedMeasurements(timestep);
            numDiscardedMeasurements = this.statistics.numDiscardedMeasurements(timestep);
            controllerState = this.statistics.controllerStates(:, timestep + 1);
        end
        
        %% changeSequenceLength
        function ret = changeSequenceLength(this, newSequenceLength)
            % Change the length of the control sequence used by the
            % employed controller.
            % This operation does nothing but returning false if this is not supported by the controller.
            %
            % Parameters:
            %   >> newSequenceLength (Positive integer)
            %      The new sequence length to used by the controller.
            %
            % Returns:
            %   << ret (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the sequence length of the
            %      employed controller was changed. 
            %      False is returned in case the controller does not support this.
            
            ret = false;
            if this.canChangeSequenceLength()
                this.controller.changeSequenceLength(newSequenceLength);
                ret = true;
            end
        end
        
        %% changeCaDelayProbs
        function ret = changeCaDelayProbs(this, newCaDelayProbs)
            % Change the probability distribution of the delays in the
            % controller-actuator link assumed by the employed controller. 
            % This operation does nothing but returning false if this is not supported by the controller.
            %
            % Parameters:
            %   >> newCaDelayProbs (Nonnegative vector)
            %      The new probability distribution to be assumed by the controller.
            %
            % Returns:
            %   << ret (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the probability distribution
            %      was changed.
            %      False is returned in case the controller does not support this.
            
            ret = false;
            if this.canChangeCaDelayProbs()
                % validate the given probability distribution
                Validator.validateDiscreteProbabilityDistribution(newCaDelayProbs);
                % do the change
                this.doChangeCaDelayProbs(newCaDelayProbs);
                ret = true;
            end
        end
        
        %% changeModelParameters
        function ret = changeModelParameters(this, newA, newB, newW, newC)            
            ret = false;
            if this.canChangeModelParameters()
                this.doChangeModelParameters(newA, newB, newW, newC);
                ret = true;
            end
        end
    end
    
    methods (Access = protected)
        
        %% preDoStep
        function preDoStep(this, timestep)
            % Function of the step() template method to carry out additional preprocessing 
            % prior to the computation of the control sequence.
            % This default implementation does nothing.
        end
        
        %% doStep
        function [controllerState, inputSequence, numUsedMeas, numDiscardedMeas] ...
                = doStep(this, timestep, scPackets, acPackets, plantMode)
            % Main function of the step() template method, can be overriden
            % by subclasses.
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
            %   << controllerState (Vector)
            %      The controller's estimate of the current plant state,
            %      expressed with regards to the plant coordinates.
            %
            %   << inputSequence (Matrix, might be empty)
            %      The computed control sequence, shaped as matrix, 
            %      with the individual inputs now column-wise arranged.
            %      Empty matrix is returned if no sequence was computed, e.g. in case the controller is event-based.
            %
            %   << numUsedMeas (Nonnegative integer)
            %      The number of measurements processed by the controller.
            %
            %   << numDiscardedMeas (Nonnegative integer)
            %      The number of measurements discarded by the controller due to their delays.            
            
            [measurements, measDelays] = NcsController.processScPackets(scPackets);
            [modeObservations, modeDelays] = NcsController.processAcPackets(timestep, acPackets);
            % add the previous true mode to the mode observations, if available
            if ~isempty(plantMode) && ~ismember(1, modeDelays)
                modeObservations(end + 1) = plantMode;
                modeDelays(end + 1) = 1;
            end
            
            [controllerState, inputSequence, numUsedMeas, numDiscardedMeas] ...
                = this.doStepFun(measurements, measDelays, modeObservations, modeDelays);
            % shift controller state if required, to be expressed with
            % regards to the plant coordinates
            if ~isempty(this.plantStateOrigin)
                controllerState = controllerState + this.plantStateOrigin;
            end
        end
        
        %% postDoStep
        function dataPacket = postDoStep(this, inputSequence, controllerState, timestep)
            % Function of the step() template method to carry out additional postprocessing.
            % By default, a data packet is created containing the computed
            % control sequence.
            %
            % Parameters:
            %   << inputSequence (Matrix, might be empty)  
            %      The computed control sequence, shaped as matrix, 
            %      with the individual inputs now column-wise arranged.
            %      Empty matrix is returned if no sequence was computed, e.g. in case the controller is event-based.
            %
            %   << controllerState (Vector)
            %      The controller's estimate of the current plant state,
            %      expressed with regards to the plant coordinates, as
            %      returned by the doStep() method. In this default
            %      implementation, this parameter is ignored.
            %
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %            
            % Returns:
            %   << dataPacket (DataPacket or empty matrix)
            %      The data packet containing new control sequence computed
            %      by the controller, with the individual inputs column-wise arranged,
            %      to be transmitted to the actuator.
            %      Empty matrix is returned in case none is to be transmitted (e.g., when the controller is event-based).
            
            if ~isempty(inputSequence)
                dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
            else
                dataPacket = [];
            end
        end
        
        %% recordStatistics
        function recordStatistics(this, timestep, numUsedMeas, numDiscardedMeas, controllerState)
            % Last function of the step() template method to record statistics.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            %   << numUsedMeas (Nonnegative integer)
            %      The number of measurements processed by the controller.
            %
            %   << numDiscardedMeas (Nonnegative integer)
            %      The number of measurements discarded by the controller due to their delays.
            %
            %   << controllerState (Vector)
            %      The controller's estimate of the current plant state,
            %      expressed with regards to the plant coordinates, as
            %      returned by the doStep() method. In this default
            %      implementation, this parameter is ignored.
            %
            
            this.statistics.numUsedMeasurements(timestep) = numUsedMeas;
            this.statistics.numDiscardedMeasurements(timestep) = numDiscardedMeas;
            this.statistics.controllerStates(:, timestep + 1) = controllerState;
        end       
        
        %% computeControlError
        function errorMeasure = computeControlError(this, states, timestep)
            errorMeasure = this.doComputeControlErrorFun(states, timestep);
        end
        
        %% canChangeSequenceLength
        function ret = canChangeSequenceLength(this)
            % Determine whether the sequence length of the employed
            % controller can be changed at runtime.
            %
            % Parameters:
            %   >> ret (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the sequence length of the
            %      employed controller can be changed at runtime.
            
            ret = ismethod(this.controller, 'changeSequenceLength');
        end
        
        %% canChangeCaDelayProbs
        function ret = canChangeCaDelayProbs(this)
            % Determine whether the delay distribution for the
            % controller-actuator link assumed by the controller can be
            % changed at runtime.
            % Currently, this default implementation always returns false as this
            % functionality is not yet implemented for the controllers (that don't require a filter) in use.
            % The only exception is the IMMBasedRecedingHorizonController,
            % for which functions returns true.
            %
            % Parameters:
            %   >> ret (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the probability distribution
            %      can be changed at runtime.
            
            ret = this.canChangeCaProbsFun();
        end
        
        %% canChangeModelParameters
        function ret = canChangeModelParameters(this)            
            ret = Checks.isClass(this.controller, 'IMMBasedRecedingHorizonController');
        end
        
        %% doChangeCaDelayProbs
        function doChangeCaDelayProbs(this, caDelayProbs)
            %error('NcsController:DoChangeCaDelayProbs:UnsupportedOperation', ...
            %    '** This should not happen, intended to be overriden by subclasses. **');
            this.controller.changeCaDelayProbs(caDelayProbs);
        end
        
        %% doChangeSamplingInterval
        function doChangeModelParameters(this, newA, newB, newW, ~)
            % so far only for the IMMBasedRecedingHorizonController
            this.controller.changeModelParameters(newA, newB, newW);
        end
        
        %% reshapeInputSequence
        function sequence = reshapeInputSequence(this, inputSequence)
            % Reshape a control sequence, so that the individual inputs are
            % column-wise arranged.
            %
            % Parameters:
            %   >> inputSequence (Column Vector, might be empty)
            %      A vector containing the stacked control inputs constituting a sequence.
            %
            % Returns:
            %   << sequence (Matrix, might be empty)
            %      The passed control sequence, reshaped as matrix, 
            %      with the individual inputs now column-wise arranged.
            %      Empty matrix is returned if inputSequence is empty.
            
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
            % Create a data packet containing the given control sequence to
            % be sent to the plant/actuator.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            %   >> inputSequence (Matrix)
            %      The control sequence to be transmitted, given as a nonempty
            %      matrix, with the individual inputs column-wise arranged.
            %
            % Returns:
            %   << dataPacket (DataPacket)
            %      The resulting data packet to be transmitted to the actuator.
             
            % the control sequence is transmitted from the controller (id = 2) to the actuator (id = 1)
            dataPacket = CreateDataPacket(inputSequence, timestep, 2, 1);
         end
        

        %% processAcPackets
        function [modeObservations, modeDelays] = processAcPackets(timestep, acPackets)
            % Extract the plant modes and corresponding delays from the
            % acknowledgment packets received from the actuator. 
            %
            % Parameters:
            %   >> acPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing acknowledgments received from the actuator.
            %
            % Returns:
            %   << modeObservations (Matrix, might be empty)
            %      The mode observations extracted from the data packets, column-wise arranged.
            %      Empty matrix is returned if acPackets is empty.
            %
            %   << modeDelays (Vector, might be empty)
            %      A vector containing the delays of the received mode observation (in timesteps), 
            %      where the i-th element denotes the delay of the i-th mode observation.
            %      Empty matrix is returned if acPackets is empty.
            
            modeObservations = [];
            modeDelays = [];
            if numel(acPackets) ~= 0
                % get the observed modes from the ACK packets
                modeObservations = zeros(1, numel(acPackets));
                modeDelays = zeros(1, numel(acPackets));
                for i=1:numel(acPackets)
                    modeTime = acPackets(i).payload{2}.timeStep; % the time step of the mode piggy-backed by the ack
                    modeObservations(i) = acPackets(i).payload{2}.theta; % the value of the mode, given as integer in [1, controlSeqLength + 1]
                    modeDelays(i) = timestep - modeTime;
                end
            end
        end
         
        %% processScPackets
        function [measurements, measDelays] = processScPackets(scPackets)
            % Extract the measurements and corresponding delays from the
            % data packets received from the sensor. 
            %
            % Parameters:
            %   >> scPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing measurements taken and transmitted from the sensor.
            %
            % Returns:
            %   << measurements (Matrix, might be empty)
            %      The measurements extracted from the data packets,
            %      column-wise arranged.
            %      Empty matrix is returned if scPackets is empty.
            %
            %   << measDelays (Vector, might be empty)
            %      A vector containing the delays of the received measurements (in timesteps), 
            %      where the i-th element denotes the delay of the i-th measurement.
            %      Empty matrix is returned if scPackets is empty.
            
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
    
    methods (Access = private)
        
        %% constructCanChangeCaDelayProbsFun
        function fun = constructCanChangeCaDelayProbsFun(this)
            if Checks.isClass(this.controller, 'IMMBasedRecedingHorizonController')
                fun = @() true;
            else
                fun = @() false;
            end
        end
        
        %% constructComputeControlErrorFun
        function fun = constructComputeControlErrorFun(this)
            % use an integral measure, i.e., a trailing sum with a fixed
            % horizon (number of columns of states) that is considered
            if Checks.isClass(this.controller, 'SequenceBasedTrackingController')
                % we track a reference of arbitrary dimension
                fun=@computeTrackingError;
            else
                % we drive the plant to the origin
                %fun = @(states, timestep) sum(sqrt(sum(states .^ 2)));
                fun = @(states, timestep) sum(vecnorm(states));
            end
            
            %% computeTrackingError
            function trackingError = computeTrackingError(states, timestep)
                trackingError = 0;
                timesteps = timestep - [size(states, 2)-1:-1:0];
                for i=1:numel(timesteps)
                    trackingError = trackingError + norm(this.controller.getDeviationFromRefForState(states(:, i), timesteps(i)));
                end
            end
            
        end
        
        %% constructDoStepFun
        function fun = constructDoStepFun(this)
            switch metaclass(this.controller)
                case {?IMMBasedRecedingHorizonController}
                    disp('hit');
                    fun = @computeGetState;
                otherwise
                    fun = @getStateCompute;
            end
            
            function [state, input, numUsedMeas, numDiscardedMeas] = getStateCompute(measurements, measDelays, modeObservations, modeDelays)
                % retrieve state before sequence is computed
                state = this.controller.getControllerPlantState();
                input = this.reshapeInputSequence(this.controller.computeControlSequence(measurements, measDelays, modeObservations, modeDelays));
                [numUsedMeas, numDiscardedMeas] = this.controller.getLastComputationMeasurementData();
            end
            function [state, input, numUsedMeas, numDiscardedMeas] = computeGetState(measurements, measDelays, modeObservations, modeDelays)
                input = this.reshapeInputSequence(this.controller.computeControlSequence(measurements, measDelays, modeObservations, modeDelays));
                % retrieve state after sequence is computed
                state = this.controller.getControllerPlantState();
                [numUsedMeas, numDiscardedMeas] = this.controller.getLastComputationMeasurementData();
            end            
        end
    end
end

