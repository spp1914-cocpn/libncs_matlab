classdef NetworkedControlSystem < handle
    % This class represents a single networked control system, consisting
    % of (linear) plant and sensor, an actuator, and a controller.
    
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
    
    properties (GetAccess = public, SetAccess = immutable)
        name = 'NCS';        
        plantSamplingInterval(1,1) double {mustBePositive} = NetworkedControlSystem.defaultPlantSamplingTime;
        networkType NetworkType = NetworkType.UdpLikeWithAcks;
        sensor NcsSensor;
        controller NcsController;
        plant NcsPlant;
    end
    
    properties (GetAccess = public, SetAccess = private)
        % controller can change sampling rate during runtime
        samplingInterval(1,1) double {mustBePositive} = NetworkedControlSystem.defaultSamplingTime; % in seconds, all components are clock-synchronous
    end
    
    properties (Access = private)        
        translator NcsTranslator;
        lastPlantInvocationTime; % stores the last point in time (in pico-seconds) the plant was triggered from Omnet
        lastControllerInvocationTime; % stores the last point in (in pico-seconds) the controller/actuator was triggered from Omnet
        lastControllerTimestep = 0; % store the time step corresponding to the last invocation time, needed to keep track of since controller can adapt sampling rate
        
        interpolatedPlantStates;
        inputs;
        plantMode; % updated periodically, when actuator is triggered
        
        avgEstimatedControlError(1,1) double = 0;
        avgActualControlError(1,1) double = 0;        
        
        lastEstimatedControlError(1,1) double = 0;
        lastActualControlError(1,1) double =0;
        numControlErrorSamples(1,1) double = 0;      
    end
    
    properties (Dependent, GetAccess = public)
        controlSequenceLength;
    end
    
    properties (Constant, Access = private)
         % which is a frequency of 10 Hz
        defaultSamplingTime = 0.1;
        defaultPlantSamplingTime = 0.001; % 1 kHz
    end
    
    methods
        function sequenceLength = get.controlSequenceLength(this)
            sequenceLength = this.controller.controlSequenceLength;
        end
    end
       
    methods (Access = public)
        %% NetworkedControlSystem
        function this = NetworkedControlSystem(controller, plant, sensor, name, samplingInterval, plantSamplingInterval, networkType)
            % Class constructor.
            %
            % Parameters:
            %   >> controller (NcsController)
            %      The controller of the NCS.
            %
            %   >> plant (NcsPlant)
            %      The plant of the NCS.
            %
            %   >> plant (NcsSensor)
            %      The sensor of the NCS.
            %
            %   >> name
            %      Name or identifier for NCS.
            %
            %   >> samplingInterval (Positive Scalar)
            %      A positive scalar denoting the sampling interval (in
            %      seconds) of the NCS (controller, actuator and sensor). If none is passed, 0.1 is used.
            %
            %   >> plantSamplingInterval (Positive Scalar, optional)
            %      A positive scalar denoting the sampling interval (in
            %      seconds) of the plant. If none is passed, 0.001 is used.
            %
            %   >> networkType (NetworkType, optional)
            %      A constant from the enumeration describing the network
            %      type (e.g. TCP-like) that shall be used for this
            %      instance.
            %      If none is passed NetworkType.UdpLikeWithAcks is used.
            %
            % Returns:
            %   << this (NetworkedControlSystem)
            %      A new NetworkedControlSystem instance.
            
            if nargin > 4
                this.samplingInterval = samplingInterval;
            end            
            if nargin > 5
                this.plantSamplingInterval = plantSamplingInterval;
            end
            if nargin > 6
                this.networkType = networkType;
            end            
            this.name = name;
            this.controller = controller;
            this.plant = plant;
            this.sensor = sensor;
            
            this.plantMode = this.controlSequenceLength + 1;            
        end
        
        %% attachTranslator
        function attachTranslator(this, translator)
            this.translator = translator;
        end
                
        %% evaluateRateQualityCharacteristics
        function [rate, rateChange] = evaluateRateQualityCharacteristics(this, actualQoc, targetQoc)
            % Get the data rate required to reach a target control performance (QoC) given an actual QoC
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> actualQoc (Nonnegative scalar)
            %      The current actual QoC of the NCS.
            %
            %   >> targetQoc (Nonnegative scalar)
            %      The desired QoC of the NCS.
            %
            % Returns:
            %   << rate (Nonnegative scalar)
            %      The data rate (in packets per second) required to
            %      reach the target QoC, based on the underlying communication
            %      characteristics. The maximum value to be returned is
            %      1/samplingRate.
            %
            %   << rateChange (Nonnegative scalar, optional)
            %      The partial derivative of the underlying communication
            %      model with regards to the target QoC, evaluated at the
            %      given value.
            
            this.checkTranslator();
            if nargout == 1
                rate = this.translator.getDataRateForQoc(actualQoc, targetQoc);
            else
                [rate, rateChange] = this.translator.getDataRateForQoc(actualQoc, targetQoc);
            end
        end
        
        %% evaluateQualityRateCharacteristics
        function qoc = evaluateQualityRateCharacteristics(this, actualQoc, targetRate)
            % Get the control performance (QoC) that can be achieved, given an actual QoC and a desired data rate,
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> actualQoc (Nonnegative scalar)
            %      The current actual QoC of the NCS.
            %
            %   >> targetRate (Nonnegative scalar)
            %      The desired data rate (in packets per second).
            %
            % Returns:
            %   << qoc (Nonnegative scalar)
            %      The QoC that can be achieved given the values of actual QoC and desired data rate,
            %      computed based on the underlying communication characteristics of the NCS.
             
            this.checkTranslator();
            qoc = this.translator.getQocForDataRate(actualQoc, targetRate);
        end
        
        %% initPlant
        function initPlant(this, plantState)
            % Set the initial plant state.
            %
            % Parameters:
            %   >> plantState (Vector or Distribution)
            %      A vector or a probability distribution of appropriate dimension, i.e., the size
            %      equals the one expected by the plant.
            %      If a distribution is passed, the initial state is
            %      randomly drawn according to the given probability law.
            %
            
            if isa(plantState, 'Distribution')
                state = plantState.drawRndSamples(1);
            else
                state = plantState;
            end
            assert(isnumeric(state) && isvector(state) && length(state) == this.plant.dimState && all(isfinite(state)), ...
                'NetworkedControlSystem:InitPlant', ...
                '** Cannot init plant: State must be a real-valued %d-dimensional vector or distribution **', this.plant.dimState);
            
            this.plant.init(state(:));            
        end
        
        %% initStatisticsRecording
        function initStatisticsRecording(this, maxSimTime)
            % Initialize the recording of data to be gathered at each
            % time step, which are: true state, true mode, applied input,
            % number of used measurements, number of discarded
            % measurements, number of discarded control sequences, controller state.
            % 
            % In particular, memory is allocated for the given number of
            % time steps and the initial data (at k=0) are recorded.
            %
            % Parameters:
            %   >> maxSimTime (Positive Scalar)
            %      A positive scalar denoting the maximum simulation time in pico-seconds. 
            
            arguments
                this
                maxSimTime(1,1) {mustBeNumeric, mustBePositive, mustBeInteger}
            end 
            
            % controller can adapt its sampling rate, so the following is a
            % mere educated guess
            maxControllerSteps = floor(ConvertToSeconds(double(maxSimTime)) / this.samplingInterval);
            maxPlantSteps = floor(ConvertToSeconds(double(maxSimTime)) / this.plantSamplingInterval);
      
            % controller and actuator are synchronized
            this.controller.initStatisticsRecording(maxControllerSteps, this.plant.dimState);
            this.plant.initStatisticsRecording(maxPlantSteps, maxControllerSteps);
            
            this.interpolatedPlantStates = nan(this.plant.dimState, maxControllerSteps + 1);
            this.interpolatedPlantStates(:, 1) = this.plant.getInitialPlantState();            
            this.inputs = nan(this.plant.dimInput, maxControllerSteps);
        end
        
        %% getStatistics
        function statistics = getStatistics(this)
            % Get the statistical data that has been recorded during a
            % simulation run.
            %
            % Returns:
            %   << statistics (Struct)
            %      The statistical data.
            %
            
            statistics = this.plant.getStatistics(this.lastControllerTimestep);
            controllerStats = this.controller.getStatistics(this.lastControllerTimestep);            
            fields = fieldnames(controllerStats);
            for i=1:length(fields)
                statistics.(fields{i}) = controllerStats.(fields{i});
            end
        end
        
        %% getStageCosts
        function currentStageCosts = getCurrentStageCosts(this)
            % Get the true stage costs for the current controller time step.            
            %
            % Returns:
            %   << stageCosts (Nonnegative scalar)
            %      The current stage costs according to the controller's underlying cost functional.
            
            if isempty(this.lastPlantInvocationTime) || isempty(this.lastControllerInvocationTime)
                currentStageCosts = 0;
            
            else
                % use stored x_k and u_k
                currentStageCosts = this.controller.getCurrentStageCosts(this.interpolatedPlantStates(:, this.lastControllerTimestep + 1), ...
                    this.inputs(:, this.lastControllerTimestep), this.lastControllerTimestep); 
            end
        end
        
        %% getQualityOfControl
        function [actualQoc, estimatedQoc] = getCurrentQualityOfControl(this)
            % Get the current quality of control (QoC), which is an application specific measure of the control performance.
            %
            % Returns:
            %   << actualQoc (Nonnegative scalar)
            %      The current quality of control, computed based on the
            %      control error using an application specific mapping.
            %      In general, the returned value is nonnegative and such
            %      that a large value corresponds to a high control performance.
            %
            %   << estimatedQoc (Nonnegative scalar)
            %      The current quality of control, as estimated/perceived by the controller, computed based on the
            %      estimated/perceived control error using an application specific mapping.
            %      In general, the returned value is nonnegative and such
            %      that large a value corresponds to a high control performance.
            
            if isempty(this.lastPlantInvocationTime) || isempty(this.lastControllerInvocationTime)...
                    || isempty(this.translator)
                actualQoc = 0;
                estimatedQoc = 0;
            else
                % qoc is in unit interval [0,1]
                actualQoc = this.translator.translateControlError(this.avgActualControlError);
                estimatedQoc = this.translator.translateControlError(this.avgEstimatedControlError);
            end
        end
        
        %% getControlError
        function [actualError, estimatedError] = getCurrentControlError(this)
            % Get the current control error.
            %
            % Returns:
            %   << actualError (Nonnegative scalar)
            %      The current control error, which is an integral measure: the accumulated norm of the 
            %      plant true states at times k, k-1, ..., k-9, (with respect to the controller plant model), or, in a tracking task, the norm of
            %      the accumulated norm of the deviation from the reference
            %      output or setpoint at times k, k-1, ..., k-9.
            %
            %   << estimatedError (Nonnegative scalar)
            %      The current control error, which is an integral measure, as estimated/perceived by the controller: the accumulated norm of the 
            %      plant states as estimated by the controller at times k, k-1, ..., k-9, (with respect to the controller plant model), or, in a tracking task, the norm of
            %      the accumulated norm of the deviation from the reference
            %      output or setpoint at times k, k-1, ..., k-9.
                       
            if isempty(this.lastPlantInvocationTime) || isempty(this.lastControllerInvocationTime)
                actualError = 0;
                estimatedError = 0;
            else
                estimatedError = this.lastEstimatedControlError;
                actualError = this.lastActualControlError;
            end
        end        
        
        %% isPlantStateAdmissible
        function admissible = isPlantStateAdmissible(this)
            % Get whether the current plant state is admissible (e.g., does not violate constraints).
            %
            % Returns:
            %   << admissible (Logical scalar i.e., a flag)
            %      True in case the current plant state is admissible,
            %      false otherwise.
            
            if isempty(this.lastPlantInvocationTime)
                admissible = true;
            else
                admissible = this.plant.isStateAdmissible();
            end
        end
        
        %% computeTotalControlCosts
        function costs = computeTotalControlCosts(this)
            % Compute accrued costs of the control task
            % according to the cost functional of the employed controller.
            %
            % Returns:
            %   << costs (Nonnegative scalar)
            %      The accrued costs according to the cost functional of the employed controller.                     
            
            costs = this.controller.computeCosts(this.interpolatedPlantStates, this.inputs);
        end
        
        %% plantStep
        function plantStep(this, timestamp)     
            
            this.plant.plantStep(this.getPlantTimestep(timestamp));              
            this.lastPlantInvocationTime = timestamp;
        end
        
        %% step
        function [controllerActuatorPacket, sensorControllerPacket, controllerAcks] ...
                = step(this, timestamp, scPackets, caPackets, acPackets)
            % Execute a single time-triggered control cycle as described on
            % pages 36-37 in: 
            %   Jörg Fischer,
            %   Optimal sequence-based control of networked linear systems,
            %   Karlsruhe series on intelligent sensor-actuator-systems, Volume 15,
            %   KIT Scientific Publishing, 2015.
            %
            % Parameters:
            %   >> timestamp (Positive integer)
            %      The current simulation time (in Omnet), in pico-seconds,
            %      being an integer multiple of the sampling interval.
            %     
            %   >> scPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing measurements taken and transmitted from the sensor.
            %
            %   >> caPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing control sequences sent from the controller.
            %
            %   >> acPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing ACKs returned from the actuator.
            %
            % Returns:
            %   << controllerActuatorPacket (DataPacket or empty matrix)
            %      The data packet containing new control sequence computed
            %      by the controller, with the individual inputs column-wise arranged,
            %      to be transmitted to the actuator.
            %      Empty matrix is returned in case none is to be transmitted (e.g., when the controller is event-based).
            %
            %   << sensorControllerPacket (DataPacket or empty matrix)
            %      The data packet containing the measurement to be transmitted to the controller.
            %      Empty matrix is returned in case none is taken or to be transmitted (e.g., when the sensor is event-based).
            %
            %   << controllerAcks (Empty matrix or column vector of DataPackets)
            %      The actuator creates an acknowledgment packet for each
            %      received control sequence in case the underlying network
            %      type is UdpLikeWithAcks. Empty in case TcpLike or UdpLike
            %      network type is used.
            
            %controllerTimestep = this.getTimestep(timestamp)
            controllerTimestep = this.lastControllerTimestep + 1;
            portion = (double(timestamp) - this.lastPlantInvocationTime) / ConvertToPicoseconds(this.plantSamplingInterval);
            
            if ~Checks.isScalarIn(portion, 0,1)
                warning('NetworkedControlSystem:Step:StrangePortion', ...
                    '** This should not happen to often: value of <portion> was %f **', portion);
            end
            
            % get the current plant input, which is simply the one used at
            % the last plant step (piecewise constant inputs applied)
            lastPlantTimeStep = this.getPlantTimestep();
            [~, actualInput] = this.plant.getPlantStatsForTimestep(lastPlantTimeStep); % u_k
            this.inputs(:, controllerTimestep) = actualInput; 
            
            interpolatedState = this.plant.getInterpolatedPlantState(lastPlantTimeStep, portion);
            this.interpolatedPlantStates(:, controllerTimestep + 1) = interpolatedState;            
            % take a measurement y_k
            sensorControllerPacket = this.sensor.step(controllerTimestep, interpolatedState);
                                   
            % do not pass the previous plant mode to the controller unless
            % network is TCP-like
            previousMode = [];
            if this.networkType.previousPlantModeAvailable()
                 previousMode = this.plantMode;
            end
            
            % set the packet delays
            for p = scPackets
                p.packetDelay = controllerTimestep - p.timeStamp;
            end
            for p = acPackets
                p.packetDelay = controllerTimestep - p.timeStamp;
            end
            for p = caPackets
                p.packetDelay = controllerTimestep - p.timeStamp;
            end
            
            controllerActuatorPacket = this.controller.step(controllerTimestep, scPackets, acPackets, previousMode);         
                              
            %this.plantState = newPlantState;
            [this.plantMode, controllerAcks] = this.plant.actuatorStep(controllerTimestep, caPackets, this.networkType.sendOutAck());             
            
            this.lastControllerInvocationTime = timestamp;
            this.lastControllerTimestep = this.lastControllerTimestep + 1;
            
            % compute the most current control error (actual and perceived value)
            [this.lastEstimatedControlError, this.lastActualControlError] ...
                = this.controller.getCurrentControlError(this.lastControllerTimestep, ...
                        this.interpolatedPlantStates(:, 1:this.lastControllerTimestep+1));
            
            if ConvertToSeconds(timestamp) > 10 % hardcoded
                % contributes to the average values
                N = this.numControlErrorSamples + 1;               
                
                this.avgActualControlError = ((N-1) * this.avgActualControlError + this.lastActualControlError) / N;
                this.avgEstimatedControlError = ((N-1) * this.avgEstimatedControlError + this.lastEstimatedControlError) / N;
                
                this.numControlErrorSamples = N;
            end
        end
        
        %% changeControllerSequenceLength
        function success = changeControllerSequenceLength(this, newSequenceLength)
            % Change the length of the control sequence used by the
            % controller in the NCS. This operation does nothing if this is
            % not supported by the controller.
            %
            % Parameters:
            %   >> newSequenceLength (Positive integer)
            %      The new sequence length to used by the controller.
            %
            % Returns:
            %   << success (Logical Scalar, i.e., a boolean)
            %      A flag indicating whether the sequence length of the
            %      employed controller was changed. 
            %      False is returned in case the controller does not support this.
                      
            success = this.controller.changeSequenceLength(newSequenceLength);
            if success
                % requires that the actuator also changes its sequence
                % length
                this.plant.changeActuatorSequenceLength(newSequenceLength);
            end
        end
        
        %% changeControllerCaDelayProbs
        function success = changeControllerCaDelayProbs(this, newCaDelayProbs)
            % Change the probability distribution of the delays in the
            % controller-actuator link assumed by the controller in the NCS. 
            % This operation does nothing if this is not supported by the controller.
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
                        
            success = this.controller.changeCaDelayProbs(newCaDelayProbs);
        end
        
        %% changeControllerSamplingInterval
        function success = changeControllerSamplingInterval(this, newSamplingInterval)
            success = false;
            if newSamplingInterval ~= this.samplingInterval
                % check the plant, so far only InvertedPendulum is
                % supported
                if Checks.isClass(this.plant.plant, 'InvertedPendulum')
                    % we have to adapt the linearization
                    % measurement matrix C does not change
                    [A_new, B_new, ~, W_new] = this.plant.plant.linearizeAroundUpwardEquilibrium(newSamplingInterval);
                    success = this.controller.changeModelParameters(A_new, B_new, W_new, []);
                end
            end
            if success
                this.samplingInterval = newSamplingInterval;
            end
        end
    end
    
    methods(Access = private)        
        
        %% getPlantTimestep
        function plantTimestep = getPlantTimestep(this, timestamp)
            if nargin == 1
                plantTimestep = round(this.lastPlantInvocationTime / ConvertToPicoseconds(this.plantSamplingInterval));
            else
                % we are given a timestamp (in pico-seconds)
                % functions are triggered periodically during simulations,
                % so given timestamp should always be an integer multiple
                % of the plant sampling interval (in practice, it might not
                % be due to numerical issues and weird sampling frequencies such as 95 Hz)
                plantTimestep = round(timestamp / ConvertToPicoseconds(this.plantSamplingInterval));
            end
        end
        
        %% getTimestep
        function timestep = getTimestep(this, timestamp)
            if nargin == 1
                timestep = round(this.lastControllerInvocationTime / ConvertToPicoseconds(this.samplingInterval));
            else
                % we are given a timestamp (in pico-seconds)
                % functions are triggered periodically during simulations,
                % so given timestamp should always be an integer multiple
                % of the sampling interval (in practice, it might not
                % be due to numerical issues and weird sampling frequencies such as 95 Hz)
                timestep = round(timestamp / ConvertToPicoseconds(this.samplingInterval));
            end
        end
        
        %% checkSensor
        function checkSensor(this)
            assert(~isempty(this.sensor), ...
                'NetworkedControlSystem:CheckSensor', ...
                '** Sensor has not been specified **');
        end
        
        %% checkTranslator
        function checkTranslator(this)
            assert(~isempty(this.translator), ...
                'NetworkedControlSystem:CheckTranslator', ...
                '** CoCPN-Translator has not been specified **');
        end
    end
end

