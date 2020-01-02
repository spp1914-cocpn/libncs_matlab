classdef NetworkedControlSystem < handle
    % This class represents a single networked control system, consisting
    % of (linear) plant and sensor, an actuator, and a controller.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2019  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        name;
        samplingInterval; % in seconds, all components are clock-synchronous
        networkType@NetworkType;
        sensor@NcsSensor;
        controller@NcsController;
        plant@NcsPlant;
    end
    
    properties (Access = private)
        plantState;
        plantMode;       
        translator@NcsTranslator;
    end
    
    properties (Dependent, GetAccess = public)
        controlSequenceLength;
    end
    
    properties (Constant, Access = private)
         % which is a frequency of 10 Hz
        defaultSamplingTime = 0.1;
    end
    
    methods
        function sequenceLength = get.controlSequenceLength(this)
            sequenceLength = this.controller.controlSequenceLength;
        end
    end
       
    methods (Access = public)
        %% NetworkedControlSystem
        function this = NetworkedControlSystem(controller, plant, sensor, name, samplingInterval, networkType)
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
            %   >> name (optional)
            %      Name or identifier for NCS.
            %
            %   >> samplingInterval (Positive Scalar, optional)
            %      A positive scalar denoting the sampling interval (in
            %      seconds) of the NCS. If none is passed, 0.1 is used.
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
            switch nargin
                case 3
                    name = 'NCS';
                    samplingInterval = NetworkedControlSystem.defaultSamplingTime;
                    networkType = NetworkType.UdpLikeWithAcks;
                case 4
                    samplingInterval = NetworkedControlSystem.defaultSamplingTime;
                    networkType = NetworkType.UdpLikeWithAcks;
                case 5
                    assert(Checks.isPosScalar(samplingInterval), ...
                        'NetworkedControlSystem:InvalidSamplingInterval', ...
                        '** <samplingInterval> must be a positive scalar. **');
                    networkType = NetworkType.UdpLikeWithAcks;
            end
            this.controller = controller;
            this.plant = plant;
            this.sensor = sensor;
            this.name = name;
            this.samplingInterval = samplingInterval;
            if isa(networkType, 'NetworkType')
                this.networkType = networkType;
            else
                assert(isscalar(networkType) && isinteger(networkType) ...
                        && networkType > 0 && networkType <= NetworkType.getMaxId(), ...
                    'NetworkedControlSystem:InvalidNetworkType', ...
                    '** <networkType> must be a NetworkType **');
                this.networkType = NetworkType(uint8(networkType));
            end
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
            % store as column vector
            this.plantState = state(:);
            % also, set the initial true mode (maxMode)
            this.plantMode = this.controlSequenceLength + 1;%1; 
        end
        
        %% initStatisticsRecording
        function initStatisticsRecording(this, maxLoopSteps)
            % Initialize the recording of data to be gathered at each
            % time step, which are: true state, true mode, applied input,
            % number of used measurements, number of discarded
            % measurements, number of discarded control sequences, controller state.
            % 
            % In particular, memory is allocated for the given number of
            % time steps and the initial data (at k=0) are recorded.
            %
            % Parameters:
            %   >> maxLoopSteps (Positive integer)
            %      The maximum number of simulation time steps.
            %
            
            assert(Checks.isPosScalar(maxLoopSteps) && mod(maxLoopSteps, 1) == 0, ...
                'NetworkedControlSystem:InitStatisticsRecording', ...
                '** Cannot init recording of statistics: <maxLoopSteps> must be a positive integer **');
            
            this.controller.initStatisticsRecording(maxLoopSteps, this.plant.dimState);
            this.plant.initStatisticsRecording(maxLoopSteps, this.plantState, this.plantMode);
        end
        
        %% getStatistics
        function statistics = getStatistics(this)
            % Get the statistical data that has been recorded.
            %
            % Returns:
            %   << statistics (Struct)
            %      The statistical data.
            %
            
            statistics = this.plant.statistics;
            if ~isempty(statistics)
                fields = fieldnames(this.controller.statistics);
                for i=1:length(fields)
                    f = fields{i};
                    if ~isfield(statistics, f)
                        statistics.(f) = this.controller.statistics.(f);
                    end   
                end 
            end
        end
        
        %% getStageCosts
        function currentStageCosts = getStageCosts(this, timestep)
            % Get the current stage costs for the given plant
            % true state.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            % Returns:
            %   << stageCosts (Nonnegative scalar)
            %      The current stage costs according to the controller's underlying cost functional.
            
            if isempty(this.plantState)
                % happens only if plant was not (yet) initialized
                currentStageCosts = 0;
            else
                % use stored x_k and u_k
                currentStageCosts = this.controller.getCurrentStageCosts(this.plant.statistics.trueStates(:, timestep + 1), ...
                    this.plant.statistics.appliedInputs(:, timestep), timestep);
            end
        end
        
        %% getQualityOfControl
        function [actualQoc, estimatedQoc] = getQualityOfControl(this, timestep)
            % Get the current quality of control (QoC), which is an application specific measure of the control performance.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
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
            
            if isempty(this.plantState) || isempty(this.translator)
                actualQoc = 0;
                estimatedQoc = 0;
            else           
                [estimatedError, actualError] ...
                    = this.controller.getCurrentControlError(timestep, this.plant.statistics.trueStates(:, 1:timestep+1));
                % qoc is in unit interval [0,1]
                actualQoc = this.translator.translateControlError(actualError);
                estimatedQoc = this.translator.translateControlError(estimatedError);
            end
        end
        
        %% getControlError
        function [actualError, estimatedError] = getControlError(this, timestep)
            % Get the current control error.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
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
                       
            if isempty(this.plantState)
                actualError = 0;
                estimatedError = 0;
            else           
                [estimatedError, actualError] ...
                    = this.controller.getCurrentControlError(timestep, this.plant.statistics.trueStates(:, 1:timestep+1));
            end
        end        
        
        %% isPlantStateAdmissible
        function admissible = isPlantStateAdmissible(this, timestep)
            % Get whether the current plant state is admissible (e.g., does not violate constraints).
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            % Returns:
            %   << admissible (Logical scalar i.e., a flag)
            %      True in case the current plant state is admissible,
            %      false otherwise.
                       
            admissible = this.plant.isStateAdmissible(this.plant.statistics.trueStates(:, timestep+1));
        end
        
        %% computeTotalControlCosts
        function costs = computeTotalControlCosts(this)
            % Compute accrued costs of the control task
            % according to the cost functional of the employed controller.
            %
            % Returns:
            %   << costs (Nonnegative scalar)
            %      The accrued costs according to the cost functional of the employed controller.
                        
            costs = this.controller.computeCosts(this.plant.statistics.trueStates, this.plant.statistics.appliedInputs);
        end
        
        %% step
        function [controllerActuatorPacket, sensorControllerPacket, controllerAck] ...
                = step(this, timestep, scPackets, caPackets, acPackets)
            % Execute a single time-triggered control cycle as described on
            % pages 36-37 in: 
            %   Jörg Fischer,
            %   Optimal sequence-based control of networked linear systems,
            %   Karlsruhe series on intelligent sensor-actuator-systems, Volume 15,
            %   KIT Scientific Publishing, 2015.
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
            %   << controllerAck (Empty matrix or DataPacket)
            %      The ACK for the DataPacket within the given caPackets that has
            %      become active. An empty matrix is returned in case none
            %      became active.

            % take a measurement y_k
            sensorControllerPacket = this.sensor.step(timestep, this.plantState);
                                   
            % do not pass the previous plant mode to the controller unless
            % network is TCP-like
            previousMode = [];
            if this.networkType.previousPlantModeAvailable()
                 previousMode = this.plantMode;
            end
            
            controllerActuatorPacket = this.controller.step(timestep, scPackets, acPackets, previousMode);         
            
            [controllerAck, this.plantMode, newPlantState] ...
                = this.plant.step(timestep, caPackets, this.plantState);
             
            if ~this.networkType.sendOutAck()
                controllerAck = [];
            end     
                                    
            this.plantState = newPlantState;
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
    end
    
    methods(Access = private)        
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

