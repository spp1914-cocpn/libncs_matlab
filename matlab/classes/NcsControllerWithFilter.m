classdef NcsControllerWithFilter < NcsController
    % Wrapper class for controllers that require an external filter in an NCS to provide a consistent
    % interface.
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    %      
    %   Markus Jung, Florian Rosenthal, and Martina Zitterbart,
    %   CoCPN-Sim: An Integrated Simulation Environment for Cyber-Physical Systems,
    %   Proceedings of the 2018 IEEE/ACM Third International Conference on Internet-of-Things Design and Implementation (IoTDI), 
    %   Orlando, FL, USA, April 2018.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        filter; % DelayedMeasurementsFilter, checked in ctor
        plantModel(1,1) SystemModel = LinearSystemModel; 
        measModel(1,1) LinearMeasurementModel;
    end
    
    properties (SetAccess = private, GetAccess = protected)
        bufferedControlInputSequences;
        bufferedCaDelayProbs; % buffers the ca Delay probs from the current an the past N-1 time steps, where N is the sequence length
    end
    
    properties (Access = protected)
        caDelayProbsChanged = false;
    end

    properties (SetAccess = immutable, GetAccess = private)
        doUpdateEstimateFun;
    end
    
    methods (Access = public)
        %% NcsControllerWithFilter
        function this = NcsControllerWithFilter(controller, filter, plantModel, measModel, ...
                defaultInput, initialCaDelayProbs, plantStateOrigin)
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
            if nargin == 6
                plantStateOrigin = [];
            end
            this = this@NcsController(controller, defaultInput, plantStateOrigin);
            assert(Checks.isClass(filter, 'DelayedMeasurementsFilter'), ...
                'NcsControllerWithFilter:InvalidFilter', ...
                '** <filter> must be a DelayedMeasurementsFilter **');        
            this.filter = filter;
            this.plantModel = plantModel;
            this.measModel = measModel;            
            
            this.doUpdateEstimateFun = this.constructDoUpdateEstimateFun();
            
            this.bufferedControlInputSequences ...
                = repmat(this.defaultInput, [1 this.controlSequenceLength this.controlSequenceLength]);
            probs = Utility.truncateDiscreteProbabilityDistribution(initialCaDelayProbs, ...
                this.controlSequenceLength + 1);
            this.bufferedCaDelayProbs = repmat(probs, 1, this.controlSequenceLength);
        end        
    end
    
    methods (Access = protected)
        %% preDoStep
        function preDoStep(this, ~)
            if this.canChangeCaDelayProbs() && ~this.caDelayProbsChanged % has not been triggered from outside
                % so 'change' manually
                this.doChangeCaDelayProbs(this.bufferedCaDelayProbs(:, 1));
            end
        end
        
        %% doStep
        function [controllerState, inputSequence, numUsedMeas, numDiscardedMeas] ...
                = doStep(this, timestep, scPackets, acPackets, plantMode)
            % we make use of a dedicated filter to obtain the state
            % estimate
            
            try
                % first, update the estimate, i.e., obtain x_k                
                [numUsedMeas, numDiscardedMeas, previousMode] = this.doUpdateEstimateFun(scPackets, acPackets, timestep, plantMode);
            catch ex
                % an error occurred during the filter step
                warning('NcsControllerWithFilter:DoStep:ErrorInUpdateStateEstimate', ...
                    '** An error occurred during the update of the controller state:\n\n%s\n\nSimulation can proceed but error flag will be set and no input sequence will be computed **', ...
                    ex.getReport('basic', 'hyperlinks', 'off'));
                inputSequence = [];
                controllerState = this.getControllerState();
                numUsedMeas = 0;
                numDiscardedMeas = 0;
                this.errorOccurred = true;
                return
            end
            % if previous true mode is unknown, some sort of certainty equivalence: use mode estimate
            % instead of true mode, so check that either is present
            assert(~isempty(previousMode) || ~isempty(plantMode), ...
                [class(this) ':Step:MissingPreviousPlantMode'], ...
                '** Cannot compute U_k: Neither previous plant mode nor its estimate present **');
            % compute the control inputs u_k, ..., u_{k+N}, i.e, sequence U_k
            % use (previous!) true mode (theta_{k-1}) or estimated mode of augmented system
            % and use xhat_k
            inputSequence = this.computeControlInputSequence(previousMode, timestep);
            controllerState = this.getControllerState();
        end
        
        %% postDoStep
        function dataPacket = postDoStep(this, inputSequence, controllerState, timestep)
            % finally, create the data packet, if sequence was created           
            % update the buffer for the filter
            if ~isempty(inputSequence)
                dataPacket = NcsController.createControlSequenceDataPacket(timestep, inputSequence);
            else
                inputSequence = repmat(this.defaultInput, 1, this.controlSequenceLength);
                dataPacket = [];
            end
            this.updateControlSequencesBuffer(inputSequence);
           
            this.caDelayProbsChanged = false; % reset the flag (has been set in preDoStep)
        end
        
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
        
        %% updateControlSequencesBuffer
        function updateControlSequencesBuffer(this, controlSequenceToBuffer)
            % Prepend a control sequence to the buffer of previously computed
            % and sent sequences, that still contain applicable inputs.
            %
            % Parameters:
            %   >> controlSequenceToBuffer (Matrix)
            %      A matrix denoting the newly computed control input sequence, where the
            %      elements of the sequence are column-wise arranged, to
            %      be added (prepended) to the set of still
            %      applicable sequences.
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
            ret = canChangeCaDelayProbs@NcsController(this) || Checks.isClass(this.controller, 'NominalPredictiveController');
        end
        
        %% doChangeCaDelayProbs
        function doChangeCaDelayProbs(this, caDelayProbs)            
            % given probability distribution has already been validated, so
            % adapt to number of modes
            probs = Utility.truncateDiscreteProbabilityDistribution(caDelayProbs, ...
                this.controlSequenceLength + 1);
            
            % circularly shift all columns to the right by 1, leave rows in order
            this.bufferedCaDelayProbs = circshift(this.bufferedCaDelayProbs, [0, 1]);
            this.bufferedCaDelayProbs(:, 1) = probs; % the delay distribution of the current timestep
            % the transition matrix depends on all previous delay
            % distributions
            newTransitionMatrix = Utility.calculateDelayTransitionMatrix(this.bufferedCaDelayProbs);

            if Checks.isClass(this.controller, 'InfiniteHorizonController')
                this.controller.changeTransitionMatrix(newTransitionMatrix);
            elseif Checks.isClass(this.controller, 'CaDelayProbsChangeable')
                this.controller.changeCaDelayProbs(caDelayProbs);
            end
            
            if Checks.isClass(this.filter, 'ModeTransitionMatrixChangeable')
                % adapt the transition matrix (mode theta_k) used by the filters
                this.filter.setModeTransitionMatrix(newTransitionMatrix);
            end
            
            this.caDelayProbsChanged = true;  % indicate that change ocurred
        end     
        
        %% doChangeModelParameters
        function doChangeModelParameters(this, newA, newB, newW)            
            % only supported for scenarios where inverted pendulum is to be
            % stabilized
            switch metaclass(this.plantModel) % the plant models used by the filter must be updated
                case ?JumpLinearSystemModel
                    sysModels = this.plantModel.modeSystemModels;
                    for j=1:this.controlSequenceLength + 1
                        % change the affected parameters of the model
                        sysModels{j}.setSystemMatrix(newA);
                        sysModels{j}.setSystemInputMatrix(newB);
                        sysModels{j}.setNoise(newW);
                    end
                case ?LinearPlant
                    this.plantModel.setSystemMatrix(newA);
                    this.plantModel.setSystemInputMatrix(newB);
                    this.plantModel.setNoise(newW);
            end         
            % and we update the controller, results in a recomputation of
            % the gain
            this.controller.changeModelParameters(newA, newB, newW);
        end
%%       
    end
    
    methods (Access = private)
        %% constructDoUpdateEstimateFun
        function fun = constructDoUpdateEstimateFun(this)
            switch class(this.filter)
                case 'DelayedModeIMMF'
                    fun = @(scPackets, acPackets, timestep, previousTruePlantMode) ...
                        this.updateEstimateDelayedModeIMMF(scPackets, acPackets, timestep, previousTruePlantMode);
                case 'DelayedKF'
                    fun = @(scPackets, acPackets, timestep, previousTruePlantMode) ...
                        this.updateEstimateDelayedKF(scPackets, acPackets, timestep, previousTruePlantMode);
                otherwise
                    fun = @(scPackets, acPackets, timestep, previousTruePlantMode) ...
                        this.updateEstimate(scPackets, timestep);
            end
        end
        
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
                if ~isempty(previousTruePlantMode) && ~ismember(1, modeDelays)
                    modeObservations(end+1) = previousTruePlantMode;
                    modeDelays(end+1)= 1;
                end
                [measurements, measDelays] = NcsController.processScPackets(scPackets);
                
                this.plantModel.setSystemInput(this.getPossibleSystemInputs());
                
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

        %% updateEstimateDelayedKF
        function [numUsedMeas, numDiscardedMeas, previousModeEstimate] = updateEstimateDelayedKF(this, scPackets, acPackets, timestep, previousTruePlantMode)
            % we need a special treatment
            if timestep > 1
                [modeObservations, modeDelays] = NcsController.processAcPackets(timestep, acPackets);
                if ~isempty(previousTruePlantMode) && ~ismember(1, modeDelays)
                    modeObservations(end+1) = previousTruePlantMode;
                    modeDelays(end+1)= 1;
                end
                [measurements, measDelays] = NcsController.processScPackets(scPackets);
                
                this.filter.setPossibleInputs(this.getPossibleSystemInputs());
                
                this.filter.step(this.plantModel, this.measModel, ...
                    measurements, measDelays, modeObservations, modeDelays);

                [numUsedMeas, numDiscardedMeas] = this.filter.getLastUpdateMeasurementData();
            else
                % initial timestep, no update required               
                numUsedMeas = 0;
                numDiscardedMeas = 0;
            end
            previousModeEstimate = this.filter.getPreviousModeEstimate();
        end
               
        %% getPossibleSystemInputs
        function possibleInputs = getPossibleSystemInputs(this)
            % distribute the possible inputs to all modes
            % include the default input for the last mode
            possibleInputs = [cell2mat(arrayfun(@(mode) this.bufferedControlInputSequences(:, mode, mode), ...
                    1:this.controlSequenceLength, 'UniformOutput', false)) this.defaultInput];
        end
        
    end   
end

