classdef NcsControllerWithFilter < NcsController
    % Wrapper class for controllers that require an external filter in an NCS to provide a consistent
    % interface.
    
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
    
    properties (SetAccess=immutable, GetAccess = public)
        filter@DelayedMeasurementsFilter;
        plantModel@SystemModel; 
        measModel@LinearMeasurementModel;
    end
    
    properties (Access = private)
        bufferedControlInputSequences;

    end
    
    properties (SetAccess = immutable, GetAccess = protected)
        defaultInput; % the default input, stored as column vector
    end
    
    methods (Access = public)
        %% NcsControllerWithFilter
        function this = NcsControllerWithFilter(controller, filter, plantModel, measModel, defaultInput, plantStateOrigin)
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
            %   << this (NcsControllerWithFilter)
            %      A new NcsControllerWithFilter instance.
            %
            if nargin == 5
                plantStateOrigin = [];
            end
            this = this@NcsController(controller, plantStateOrigin);
                    
            this.filter = filter;
            this.plantModel = plantModel;
            this.measModel = measModel;
            this.defaultInput = defaultInput(:);
            
             this.bufferedControlInputSequences ...
                = repmat(this.defaultInput, [1 this.controlSequenceLength this.controlSequenceLength]);
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
            %      Empty matrix is returned in case none is to be transmitted.
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
                        
            % finally, create the data packet, if sequence was created           
            % update the buffer for the filter
            if ~isempty(inputSequence)
                dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
            else
                inputSequence = repmat(this.defaultInput, 1, this.controlSequenceLength);
                dataPacket = [];
            end
            this.updateControlSequencesBuffer(inputSequence);
            
            if nargout == 4
                controllerState = this.getControllerState();
            end
        end
    end
    
    methods (Access = protected)
        %% getControllerState
        function controllerState = getControllerState(this)
            % Get the plant state as currently perceived by the controller, i.e., the filter's estimate of the plant state.
            %
            % Returns:
            %   << controllerState (Column vector)
            %      The controller's current estimate of the plant state.
            %      
            [controllerState, ~] = this.filter.getStateMeanAndCov();
                % shift controller state if required, to be expressed with
                % regards to the plant coordinates
                if ~isempty(this.plantStateOrigin)
                    controllerState = controllerState + this.plantStateOrigin;
                end
        end
        
        %% updateControllerState
        function [numUsedMeas, numDiscardedMeas, previousMode] = updateControllerState(this, scPackets, acPackets, timestep, plantMode)
            % Update the controller state, i.e., the estimate of the plant
            % state (x^e_{k}), by performing a combined time and measurement update
            % of the associated filter.
            %
            % Parameters:
            %   >> scPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing measurements taken and transmitted from the sensor.
            %
            %   >> acPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing ACKs returned from the actuator.
            %   
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %     
            %   >> plantMode (Nonnegative integer, might be empty)
            %      The previous true plant mode (theta_{k-1}), or the empty
            %      matrix, if not directly known to the controller.
            % 
            % Returns:
            %   << numUsedMeas (Nonnegative integer)
            %      The number of measurements actually used for the
            %      computation of the control sequence.
            %
            %   << numDiscardedMeas (Nonnegative integer)
            %      The number of measurements which were discarded due to
            %      a too large delay.
            %
            %   << previousMode (Nonnegative integer)
            %      The controller's estimate of the previous plant mode
            %      (theta_{k-1}), which is equal to the passed plantMode if provided. 
            % 
            if Checks.isClass(this.filter, 'DelayedModeIMMF')
                [numUsedMeas, numDiscardedMeas, previousMode] ...
                    = this.updateEstimateDelayedModeIMMF(scPackets, acPackets, timestep, plantMode);
            else
                [numUsedMeas, numDiscardedMeas, previousMode] = this.updateEstimate(scPackets, timestep);
            end
           
            % if previous true mode is unknown, some sort of certainty equivalence: use mode estimate
            % instead of true mode
            if ~isempty(plantMode)
                % should only happen in case of Tcp-like network
                previousMode = plantMode;
            elseif isempty(previousMode)
                 error([class(this) ':Step:MissingPreviousPlantMode'], ...
                    '** Cannot compute U_k: Neither previous plant mode nor its estimate present **');
            end
        end
        
        %% updateControlSequencesBuffer
        function updateControlSequencesBuffer(this, controlSequenceToBuffer)
            % Add a control sequence to the buffer of previously computed
            % and sent sequences, that still contain applicable inputs.            %
            %
            % Parameters:
            %   >> controlSequenceToBuffer (Optional arguments)
            %      Any optional arguments for the controller, such as current time step (e.g., if the controller's horizon is not infinite) or 
            %      the previous (estimated) mode of the controllor-actuator-plant subsystem.
            %
            this.bufferedControlInputSequences = circshift(this.bufferedControlInputSequences, 1, 3);            
            this.bufferedControlInputSequences(:,:, 1) = controlSequenceToBuffer;           
        end
        
         %% computeControlInputSequence
        function inputSequence = computeControlInputSequence(this, varargin)
            % Compute a sequence of control inputs to apply based on the most recent estimate of the associated filter.
            %
            % Parameters:
            %   >> varargin (Optional arguments)
            %      Any optional arguments for the controller, such as current time step (e.g., if the controller's horizon is not infinite) or 
            %      the previous (estimated) mode of the controllor-actuator-plant subsystem.
            %
            % Returns:
            %   << inputSequence (Matrix, might be empty)
            %      A matrix, where the elements of the sequence are column-wise arranged.
            %      The empty matrix is returned in case no sequence was
            %      created by the controller, for instance, if, in an
            %      event-triggered setting, none is to be transmitted.
            %      
            inputSequence = ...
                this.reshapeInputSequence(this.controller.computeControlSequence(this.filter.getState(), varargin{:}));
        end
        
        %% canChangeSequenceLength
        function ret = canChangeSequenceLength(this)
            % Determine whether the sequence length of the employed
            % controller can be changed at runtime.
            % Currently, this implementation always returns false as this
            % functionality is not yet implemented for the filters in use.
            %
            % Parameters:
            %   >> ret (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the sequence length of the
            %      employed controller can be changed at runtime.
            
            %ret = ismethod(this.controller, 'changeSequenceLength');
            ret = false;
        end
        
        %% canChangeCaDelayProbs
        function ret = canChangeCaDelayProbs(this)
            switch class(this.controller)
                case {'NominalPredictiveController', 'LinearlyConstrainedPredictiveController'}
                    % so far, only supported for the nominal controllers
                    ret = true;
                otherwise
                    ret = false;
            end
        end
        
        %% doChangeCaDelayProbs
        function doChangeCaDelayProbs(this, caDelayProbs)
            % given probability distribution has already been validated, so
            % adapt to number of modes
            probs = Utility.truncateDiscreteProbabilityDistribution(caDelayProbs, ...
                this.controlSequenceLength + 1);
            newTransitionMatrix = Utility.calculateDelayTransitionMatrix(probs);
            switch class(this.filter)
                case 'DelayedModeIMMF'
                    % we change the mode transition matrix of the underlying MJLS
                    this.filter.setModeTransitionMatrix(newTransitionMatrix);    
                case 'DelayedKF'
                    % we change the resulting weights for all possible inputs
                    this.plantModel.setDelayWeights(Utility.computeStationaryDistribution(newTransitionMatrix));
            end
        end
    end
    
    methods (Access = private)
        %% updateEstimate
        function [numUsedMeas, numDiscardedMeas, previousModeEstimate] = updateEstimate(this, scPackets, timestep)
            
            if timestep > 1
                this.distributePossibleSystemInputs();
                
                [measurements, measDelays] = NcsController.processScPackets(scPackets);
                if ~isempty(measurements)
                    this.filter.step(this.plantModel, this.measModel, measurements, measDelays);
                    [numUsedMeas, numDiscardedMeas] = this.filter.getLastUpdateMeasurementData();
                else
                    % only perform a prediction
                    this.filter.predict(this.plantModel);
                    numUsedMeas = 0;
                    numDiscardedMeas = 0;
                end
                % all other filters do currently not provide an appropriate
                % estimate
                previousModeEstimate = [];
            else
                % initial timestep, no update required
                previousModeEstimate = [];
                numUsedMeas = 0;
                numDiscardedMeas = 0;
            end
        end
        
        %% updateEstimateDelayedModeIMMF
        function [numUsedMeas, numDiscardedMeas, previousModeEstimate] = updateEstimateDelayedModeIMMF(this, scPackets, acPackets, timestep, previousTruePlantMode)
            % we need a special treatment
            if timestep > 1
                [modeObservations, modeDelays] = NcsController.processAcPackets(timestep, acPackets);
                if ~isempty(previousTruePlantMode) &&  ~ismember(1, modeDelays)
                    modeObservations(end+1) = previousTruePlantMode;
                    modeDelays(end+1)= 1;
                end
                [measurements, measDelays] = NcsController.processScPackets(scPackets);
                
                this.distributePossibleSystemInputs();
                
                this.filter.step(this.plantModel, this.measModel, ...
                    measurements, measDelays, modeObservations, modeDelays);

                [numUsedMeas, numDiscardedMeas] = this.filter.getLastUpdateMeasurementData();
                previousModeEstimate = this.filter.getPreviousModeEstimate();
            else
                % initial timestep, no update required
                numUsedMeas = 0;
                numDiscardedMeas = 0;
                previousModeEstimate = this.filter.getPreviousModeEstimate(false);
            end
            
        end
       
        %% distributePossibleSystemInputs
        function distributePossibleSystemInputs(this)
            % distribute the possible inputs to all modes
            modeSpecificInputs = arrayfun(@(mode) this.bufferedControlInputSequences(:, mode, mode), ...
                    1:this.controlSequenceLength, 'UniformOutput', false);
            % include the default input for the last mode 
            this.plantModel.setSystemInput([cell2mat(modeSpecificInputs) this.defaultInput]);
        end       
       
    end   
end

