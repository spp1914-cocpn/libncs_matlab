classdef NcsPlant < handle
    % Wrapper class for (linear) subsystem actuator/plant in an NCS to provide a consistent
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
    
    properties (SetAccess = immutable, GetAccess = protected)         
         actuator;% BufferingActuator
    end
    
    properties(Access = private)
        actualInput;        
        plantMode;
        plantState;
        
        statistics;  % struct of numeric data and timeseries objects, initialized in initStatisticsRecording()
        
        plantNoise; % for repeatability, draw in advance
    end
    
    properties (SetAccess = immutable, GetAccess = public)
       plant; % SystemModel, LinearPlant or NonlinearPlant usually 
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
            %   >> plant (SystemModel instance)
            %      The plant to be controlled, usually a LinearPlant or a NonlinearPlant.
            %
            %   >> actuator (BufferingActuator instance)
            %      The actuator in the NCS which applies inputs to the plant.
            %
            % Returns:
            %   << this (NcsPlant)
            %      A new NcsPlant instance.
            
            arguments
                plant (1,1) SystemModel
                actuator (1,1) BufferingActuator
            end
            
            this.plant = plant;
            this.actuator = actuator;
        end
        
        %% init
        function init(this, initialPlantState, maxPlantSteps)
            % Set the initial plant state.            
            % Also, for repeatability, the noise samples for the whole
            % simulation time (indicated by <maxPlantSteps>) are drawn (in case plant noise has ben set).            
            %
            % Parameters:
            %   >> initialPlantState (Vector of dimension <dimState>)
            %      A vector specifying the initial plant state.
            %             
            %   >> maxPlantSteps (Positive integer)
            %      A positive integer denoting the maximum number of plant steps to be carried out during the simulation.
            %
            %      Note: The actual number of plant steps carried out during a
            %      simulation can be smaller than specified by this parameter because an NCS can be finished
            %      prematurely, e.g., due to errors at runtime or a parameter in
            %      an Omnet ini-file indicating that an NCS should not be
            %      active over the whole simulation time.
            
            this.plantState = initialPlantState(:);           

            % initially, no input is present, so use default input provided
            % by actuator
            this.actualInput = this.actuator.defaultInput;
            % corresponding to last mode
            this.plantMode = this.actuator.controlSequenceLength + 1;

            % draw all plant noises in advance, for repeatability
            % needed, since number of sensor invocations (and thus number of
            % measurement noise samples) dependent on sampling rate and may
            % change during a simulation run
            if ~isempty(this.plant.noise)
               this.plantNoise = this.plant.noise.drawRndSamples(maxPlantSteps);
            end
        end
        
        %% initStatisticsRecording        
        function initStatisticsRecording(this, maxPlantSteps)
            % Initialize the recording of data to be gathered during simulation, 
            % which are: plant state, plant mode, applied input, number of discarded control sequences.
            % 
            % In particular, memory for the plant data (state, applied input) is allocated 
            % and the initial state is recorded.
            % 
            % Parameters:
            %   >> maxPlantSteps (Positive integer)
            %      A positive integer denoting the maximum number of plant steps to be carried out during the simulation.
            %
            %      Note: The actual number of plant steps carried out during a
            %      simulation can be smaller than specified by this parameter because an NCS can be finished
            %      prematurely, e.g., due to errors at runtime or a parameter in
            %      an Omnet ini-file indicating that an NCS should not be
            %      active over the whole simulation time.
            
             arguments
                this
                maxPlantSteps(1,1) {mustBeNumeric, mustBePositive, mustBeInteger}
            end 
            
            assert(~isempty(this.plantState), ...
                'NcsPlant:InitStatisticsRecording', ...
                '** Cannot init statistics recording: Ensure that init() has been called before **');
            
            % use array for true states and inputs as plant sampling rate
            % is fixed
            % also much more efficient than time series (and significantly
            % faster)
            this.statistics.trueStates = nan(this.dimState, maxPlantSteps + 1);
            this.statistics.trueStates(:, 1) = this.plantState;
            this.statistics.appliedInputs = nan(this.dimInput, maxPlantSteps);
            
            this.statistics.info = timeseries('trueModes/numDiscardedControlSequences'); % each element is a 2d column vector; [trueModes; numDiscardedControlSequences]            
        end
        
        %% getInitialPlantState
        function initialState = getInitialPlantState(this)
            % Get the initial plant state.
            % The initial plant state is set by calling init(), this method
            % errors if init() has not been called before.
            %
            % Returns:
            %   << initialState (Column vector of dimension <dimState>)
            %      The initial plant state.                    
            %
            assert(~isempty(this.plantState), ...
                'NcsPlant:GetInitialPlantState', ...
                '** Initial plant state not set: Ensure that init() has been called before **');
            
            initialState = this.statistics.trueStates(:, 1);
        end
        
        %% getPlantStatsForTimestep
        function [plantState, appliedInput] = getPlantStatsForTimestep(this, plantTimestep)
            % Get the plant data that has been recorded during a
            % simulation for a particular timestep.
            %
            % Parameters: 
            %   >> plantTimestep (Positive integer)
            %      The time step (w.r.t plant sampling interval) for which to get the recorded data.
            %
            % Returns:
            %   << plantState (Column vector of dimension <dimState>)
            %      The plant state at the given time step.
            %
            %   << appliedInput (Column vector of dimension <dimInput>)
            %      The input applied at the given timestep.            
            %
            arguments
                this
                plantTimestep(1,1) double {mustBePositive, mustBeInteger}
            end
            
            % check that time step (index of element in timeseries) is
            % existing and does not refer to "future" (if so, it is nan)
            assert(all(~isnan(this.statistics.appliedInputs(:, plantTimestep))), ...
                'NcsPlant:GetPlantStatsForTimestep:InvalidTimestep', ...
                '** Simulation has not yet reached plant timestep %d', plantTimestep);
            
            plantState = this.statistics.trueStates(:, plantTimestep + 1); % first sample is initial plant state
            appliedInput = this.statistics.appliedInputs(:, plantTimestep);
        end
        
        %% getActuatorStatsForTimestep
        function [numDiscardedControlSeq, trueMode] = getActuatorStatsForTimestep(this, actuatorTimestep)
            % Get the actuator data that has been recorded during a simulation for a particular timestep.
            %
            % Parameters: 
            %   >> actuatorTimestep (Positive integer)
            %      The time step (w.r.t controller sampling interval) for which to get the recorded data.
            %
            % Returns:
            %   << numDiscardedControlSeq (Nonnegative integer)
            %      The number of discarded control sequences at the given time step.
            %
            %   << trueMode (Positive integer)
            %      The mode of the augmented dynamical system at the given time step, 
            %      (i.e., the age of the buffered sequence), i.e., theta_k.
            %
            %   << appliedInput (Column vector of dimension <dimInput>)
            %      The input applied at the given timestep.            
            %
            arguments
                this
                actuatorTimestep(1,1) double {mustBePositive, mustBeInteger}
            end
            
            % check that time step (index of element in timeseries) is
            % existing and does not refer to "future"
            assert(this.statistics.info.Length >= actuatorTimestep, ...
                'NcsPlant:GetActuatorStatsForTimestep:InvalidTimestep', ...
                '** Simulation has not yet reached timestep %d', actuatorTimestep);
            
            info = this.statistics.info.getdatasamples(actuatorTimestep);
            numDiscardedControlSeq = info(2);
            trueMode = info(1);
        end
        
        %% getStatistics
        function stats = getStatistics(this, numPlantSteps, includeStateNorms)
            % Get the statistical data that has been recorded during a
            % simulation comprised of the given number of plant steps.
            %
            % Note: The actual number of plant steps carried out during a
            % simulation can be smaller than originally specified by the
            % parameter <maxPlantSteps> passed to init() because an NCS can be finished
            % prematurely, e.g., due to errors at runtime or a parameter in
            % an Omnet ini-file indicating that an NCS should not be
            % active over the whole simulation time.
            %
            % Parameters: 
            %   >> numPlantSteps (Positive integer)
            %      The number of plant time steps carried out during
            %      simulation.
            %
            %   >> includeStateNorms (Flag, i.e. logical scalar, optional)
            %      Flag to indicate whether the Euclidean norm of the plant
            %      states shall be included.
            %      That is, if true is passed, for each plant state x_k,
            %      ||x_k|| will be provided.
            %      If left out, the default value false is used.
            %
            % Returns:
            %   << statistics (Struct)
            %      The statistical data collected during the simulation.
            %      Empty matrix is returned in case
            %      initStatisticsRecording() has not been called before.
            %            
            if isempty(this.statistics)
                % happens only in case initStatisticsRecording() has not been
                % called before
                stats = [];
                return
            end
            
            % numPlantSteps might be smaller maxPlantSteps passed to initStatisticsRecording
            % happens in case ncs is finished prematurely during simulations
            % (due to errors or simulationRuntime parameter in Omnet ini file 
            % indicating that ncs is not simulated over the whole simulation time) 
            stats.trueStates = this.statistics.trueStates(:, 1:numPlantSteps + 1);
            stats.appliedInputs = this.statistics.appliedInputs(:, 1:numPlantSteps);
            
            % for convenience, also provide norms of the plant states, if requested
            if nargin > 2 && includeStateNorms
                stats.trueStateNorms = vecnorm(stats.trueStates);
            end
                        
            info = squeeze(this.statistics.info.Data);
            % corner case: No data have been recorded yet
            if isempty(info)
                stats.trueModes = [];
                stats.numDiscardedControlSequences = [];
            else
                stats.trueModes = info(1, :);
                stats.numDiscardedControlSequences = info(2, :);
            end
        end
        
        %% plantStep
        function newPlantState = plantStep(this, plantTimestep, currSimeTimeSec)
            % Perform a plant step. That is, the input currently buffered
            % by the actuator is applied and the plant is simulated in
            % time.
            %
            % Parameters:
            %   >> plantTimestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      plants's sampling interval.
            %
            %   >> currSimTimeSec (Positive scalar)
            %      The current simulation time (in seconds) that
            %      corresponds to the given timestep.
            %
            % Returns:
            %   << newPlantState (Column vector)
            %      The new plant state resulting from the application of
            %      the current input.
            
            assert(~isempty(this.plantState), ...
                'NcsPlant:PlantStep', ...
                '** Plant has not been initialized: Ensure that init() has been called before **');
            
            % apply the input to proceed to the next time step
            this.plant.setSystemInput(this.actualInput);            
            if ~isempty(this.plantNoise)                
                newPlantState = this.plant.systemEquation(this.plantState, this.plantNoise(:, plantTimestep));
            else
                % no noise present
                newPlantState = this.plant.systemEquation(this.plantState, []);
            end
            
            % record the data about true input and state
            this.statistics.appliedInputs(:, plantTimestep) = this.actualInput; % this input was applied at time k (u_k)            
            this.statistics.trueStates(:, plantTimestep + 1) = newPlantState; % store x_{k+1}
            
            this.plantState = newPlantState;
        end
        
        %% actuatorStep
        function [plantMode, controllerAcks] = actuatorStep(this, controllerTimestep, currSimTimeSec, caPackets, createAcks)
            % Process received packets (i.e., control sequences) from the
            % controller and feed a control input to the plant.
            %
            % Parameters:
            %   >> controllerTimestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      controller's sampling interval.
            %
            %   >> currSimTimeSec (Positive scalar)
            %      The current simulation time (in seconds) that
            %      corresponds to the given timestep.
            %      Not necessecarily an integer multiple of the given time
            %      step, because the controller (and hence the actuator) sampling rate might change
            %      during simulation runs.
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
            if nargin == 4 || createAcks
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
            % record the number of discarded control packets and the mode
            % theta_k
            this.statistics.info = ...
                this.statistics.info.addsample('Data', [this.plantMode; numDiscardedSeq], 'Time', currSimTimeSec);
        end        
        
        %% getInterpolatedPlantState
        function interpolatedState = getInterpolatedPlantState(this, plantTimestep, intervalPortion)
            % we always assume that we interpolate from the last time a
            % plant step was done into the future for a fraction of a full plant
            % interval
            % so the actual input is equal to the one stored in this.statistics.appliedInputs(:, plantTimestep)
            lastTrueState = this.statistics.trueStates(:, plantTimestep + 1); % since first sample is initial plant state
            % start from the given state and interpolate only until portion
            % is reached
            if intervalPortion == 0
                % nothing to do, so simply return state
                interpolatedState = lastTrueState; % this is the true plant state corresponding to the given time step
                return
            end
            if Checks.isClass(this.plant, 'InvertedPendulum') || Checks.isClass(this.plant, 'DoubleInvertedPendulum')
                oldSamplingInterval = this.plant.samplingInterval;
                this.plant.samplingInterval = oldSamplingInterval * intervalPortion;
                % use the actual input
                % use the noise drawn for the next time step (corner case:
                % last plant invocation done), if plant is noisy
                noise = [];
                if ~isempty(this.plantNoise)
                    if plantTimestep < size(this.plantNoise, 2)
                        noise = this.plantNoise(:, plantTimestep + 1);
                    else
                        % corner case
                        noise = this.plant.noise.drawRndSamples(1);
                    end
                end
                % interpolate the plant state by simulating it
                interpolatedState = this.plant.systemEquation(lastTrueState, noise);

                this.plant.samplingInterval = oldSamplingInterval;
            else
                this.plant.setSystemInput(this.statistics.appliedInputs(:, plantTimestep));
                interpolatedState = lastTrueState;
            end
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
            % Function to check whether the current plant state is admissible (e.g.,
            % does not violate constraints).
            %           
            % Returns:
            %   << isAdmissible (Flag, i.e., boolean)
            %      Flag to indicate whether the current plant state is admissible (e.g.,
            %      admissible with regards to contraints).
            
            % check if current true plant state is admissible
            isAdmissible = this.plant.isValidState(this.plantState);
        end
    end
    
end

