classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NetworkedControlSystemTest < matlab.unittest.TestCase
    % Test cases for NetworkedControlSystem.
    
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
    
    properties (Access = private)
        ncsDefaultName = 'NCS';
        ncsDefaultSamplingInterval = 0.1;
        ncsDefaultNetworkType = NetworkType.UdpLikeWithAcks;
        
        ncsName;
        ncsSamplingInterval;
        ncsNetworkType;
        
        maxLoopSteps;
        maxMeasDelay;
        controlSeqLength;
        
        dimX;
        dimU;
        dimY;
        A;
        B;
        C;
        W;
        V;
        Q
        R;
        plant;
        plantState;
        zeroPlantState;
        plantStateDistribution;
        zeroPlantStateDistribution;
        filter;
        actuator;
        controller;
        controllerSubsystem;
        sensorSubsystem;
        plantSubsystem;
        
        filterPlantModel;
        sensor;
        
        ncsUnderTest;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.ncsName = 'TestNcs';
            this.ncsSamplingInterval = 0.2;
            this.ncsNetworkType = NetworkType.TcpLike;
            
            this.maxLoopSteps = 100;
            this.maxMeasDelay = 2;
            this.controlSeqLength = 2;
            
            this.dimX = 3;
            this.dimU = 2;
            this.dimY = 1;
            
            this.A = 0.75 * eye(this.dimX);
            this.B = ones(this.dimX, this.dimU);
            this.C = [1 2 3];
            this.W = eye(this.dimX); % sys noise cov
            this.V = 0.1^2; % variance of the meas noise
            this.Q = 2 * eye(this.dimX);
            this.R = 0.5 * eye(this.dimU);
            this.plant = LinearPlant(this.A, this.B, this.W);
            this.sensor = LinearMeasurementModel(this.C);
            this.sensor.setNoise(Gaussian(0, this.V));
            this.sensorSubsystem = NcsSensor(this.sensor);
            
            this.zeroPlantState = zeros(this.dimX, 1); % plant state is already at the origin
            this.plantState = ones(this.dimX, 1); 
            this.plantStateDistribution = Gaussian(this.plantState, 0.5 * eye(this.dimX));
            this.zeroPlantStateDistribution = Gaussian(this.zeroPlantState, 0.5 * eye(this.dimX));
            
            this.filter = DelayedKF(this.maxMeasDelay);
            %this.filter.setState(this.plantStateDistribution);
            this.filterPlantModel = DelayedKFSystemModel(this.A, this.B, Gaussian(zeros(this.dimX, 1), this.W), ...
                this.controlSeqLength + 1, this.maxMeasDelay, [1/3 1/3 1/3]);
            
            this.actuator = BufferingActuator(this.controlSeqLength, this.maxMeasDelay, zeros(this.dimU, 1));
            this.plantSubsystem = NcsPlant(this.plant, this.actuator);
            
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.controllerSubsystem = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4]');
            
            this.ncsUnderTest = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.ncsNetworkType);
        end
    end
    
    methods (Test)
        %% testNetworkedControlSystemInvalidSamplingInterval
        function testNetworkedControlSystemInvalidSamplingInterval(this)
            expectedErrId = 'NetworkedControlSystem:InvalidSamplingInterval';
            
            invalidSamplingInterval = this; % not a scalar
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, invalidSamplingInterval), expectedErrId);
            
            invalidSamplingInterval = 0; % scalar value, but not positive
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, invalidSamplingInterval), expectedErrId);
            
            invalidSamplingInterval = 1i; % scalar value, but not a real value
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, invalidSamplingInterval), expectedErrId);
        end
        
        %% testNetworkedControlSystemInvalidNetworkType
        function testNetworkedControlSystemInvalidNetworkType(this)
            expectedErrId = 'NetworkedControlSystem:InvalidNetworkType';
            
            invalidNetworkType = this; % not a scalar
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, invalidNetworkType), expectedErrId);
            
            invalidNetworkType = 0; % integer, but not positive
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, invalidNetworkType), expectedErrId);
            
            invalidNetworkType = 1i; % scalar value, but not an integer
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, invalidNetworkType), expectedErrId);
            
            invalidNetworkType = NetworkType.getMaxId + 1; % integer, but out of bounds
            this.verifyError(@() NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, invalidNetworkType), expectedErrId);
        end
        
        %% testNetworkedControlSystemNoOptionalArguments
        function testNetworkedControlSystemNoOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem);
            
            this.verifyEqual(ncs.name, this.ncsDefaultName);
            this.verifyEqual(ncs.samplingInterval, this.ncsDefaultSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
        end
        
        %% testNetworkedControlSystemOneOptionalArgument
        function testNetworkedControlSystemOneOptionalArgument(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, this.ncsName);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsDefaultSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
        end
        
        %% testNetworkedControlSystemTwoOptionalArguments
        function testNetworkedControlSystemTwoOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
        end
        
        %% testNetworkedControlSystemThreeOptionalArguments
        function testNetworkedControlSystemThreeOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.ncsNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsNetworkType);
            
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            % pass network type as integer
            type = NetworkType.TcpLike;
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, int64(type));
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.networkType, type);     

            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
        end
%%
%%
        %% testEvaluateRateQualityCharacteristicsInvalidTranslator
        function testEvaluateRateQualityCharacteristicsInvalidTranslator(this)
            expectedErrId = 'NetworkedControlSystem:CheckTranslator';
            
            actualQoc = 0.5;
            targetQoc = 1;
            this.verifyError(@() this.ncsUnderTest.evaluateRateQualityCharacteristics(actualQoc, targetQoc), expectedErrId);
        end               
        
        %% testEvaluateRateQualityCharacteristics        
        function testEvaluateRateQualityCharacteristics(this)
            import matlab.unittest.constraints.IsScalar
            import matlab.unittest.constraints.IsGreaterThanOrEqualTo
            import matlab.unittest.constraints.IsLessThanOrEqualTo            
            
            qocRateCurve = cfit(fittype('a*x^2'), 1);
            controlErrorQocCurve = cfit(fittype('a*x^3'), 1.5);
            translator = NcsTranslator(qocRateCurve, controlErrorQocCurve, 1 / this.ncsUnderTest.samplingInterval);
            
            this.ncsUnderTest.attachTranslator(translator);
            
            actualQoc = 0.5;
            desiredQoc = 1;
            
            [dataRate, rateChange] = this.ncsUnderTest.evaluateRateQualityCharacteristics(actualQoc, desiredQoc);
            this.verifyThat(dataRate, IsScalar);
            this.verifyThat(dataRate, IsGreaterThanOrEqualTo(0));
            this.verifyThat(dataRate, IsLessThanOrEqualTo(1/this.ncsUnderTest.samplingInterval));
            this.verifyThat(rateChange, IsScalar);
            this.verifyThat(rateChange, IsGreaterThanOrEqualTo(0));
        end
%%
%%
        %% testEvaluateQualityRateCharacteristicsInvalidTranslator
        function testEvaluateQualityRateCharacteristicsInvalidTranslator(this)
            expectedErrId = 'NetworkedControlSystem:CheckTranslator';
            
            actualQoc = 0.5;
            targetRate = 50; % packets per second
            this.verifyError(@() this.ncsUnderTest.evaluateQualityRateCharacteristics(actualQoc, targetRate), expectedErrId);
        end
        
        %% testEvaluateQualityRateCharacteristics
        function testEvaluateQualityRateCharacteristics(this)
            import matlab.unittest.constraints.IsScalar
            import matlab.unittest.constraints.IsGreaterThanOrEqualTo            
            
            qocRateCurve = cfit(fittype('a*x^2'), 1);
            controlErrorQocCurve = cfit(fittype('a*x^3'), 1.5);
            translator = NcsTranslator(qocRateCurve, controlErrorQocCurve, 1 / this.ncsUnderTest.samplingInterval);
            
            this.ncsUnderTest.attachTranslator(translator);
            
            actualQoc = 0.5;
            targetRate = 50; % packets per second
            
            qoc = this.ncsUnderTest.evaluateQualityRateCharacteristics(actualQoc, targetRate);
            this.verifyThat(qoc, IsScalar);
            this.verifyThat(qoc, IsGreaterThanOrEqualTo(0));            
        end
%%
%%
        %% testInitPlantInvalidState
        function testInitPlantInvalidState(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            expectedErrId = 'NetworkedControlSystem:InitPlant';
            
            invalidState = this; % neither a Distribution, nor a vector
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState), expectedErrId);
            
            invalidState = ones(this.dimX +1, 1); % vector, but incorrect number of elements
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState), expectedErrId);
            
            invalidState = ones(this.dimX, 1); % vector, but contains inf
            invalidState(end) = inf;
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState), expectedErrId);
            
            invalidState = 'state'; % vector, but no numerical value
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState), expectedErrId);
            
            invalidState = Gaussian(ones(this.dimX +1, 1), eye(this.dimX + 1)); % Distribution, but wrong dimension
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState), expectedErrId);
        end
%%
%%
        %% testInitStatisticsRecordingInvalidMaxLoopSteps
        function testInitStatisticsRecordingInvalidMaxLoopSteps(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
             
            expectedErrId = 'NetworkedControlSystem:InitStatisticsRecording';
            
            invalidMaxLoopSteps = this; % not a scalar
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(invalidMaxLoopSteps), expectedErrId);
            
            invalidMaxLoopSteps = 0; % scalar value, but not positive
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(invalidMaxLoopSteps), expectedErrId);
            
            invalidMaxLoopSteps = 1i; % scalar value, but not a real value
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(invalidMaxLoopSteps), expectedErrId);
            
            invalidMaxLoopSteps = 42.5; % scalar value, but not an integer
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(invalidMaxLoopSteps), expectedErrId);
            
            invalidMaxLoopSteps = nan; % scalar value, but not finite
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(invalidMaxLoopSteps), expectedErrId);
        end
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
                 
            this.ncsUnderTest.initPlant(this.plantState);
                        
            % should be a successful call
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            stats = this.ncsUnderTest.getStatistics();
            this.verifyNotEmpty(stats);
            recordedIntialState = stats.trueStates(:, 1);
            recordedInitialMode = stats.trueModes(1);
            
            this.verifyEqual(recordedIntialState, this.plantState);
            this.verifyEqual(recordedInitialMode, this.controlSeqLength + 1);
         
        end
                
        %% testGetStatistics
        function testGetStatistics(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.ncsUnderTest.initPlant(this.plantState);
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            
            actualStatistics = this.ncsUnderTest.getStatistics();
            
            this.verifyTrue(isstruct(actualStatistics));
            
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
        end
                
         %% testGetStageCostsNotInitialized
        function testGetStageCostsNotInitialized(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            % plant was set, but not initialized
            % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getStageCosts(1), 0);
            this.verifyEqual(this.ncsUnderTest.getStageCosts(10), 0);
        end
        
        %% testGetStageCosts
        function testGetStageCosts(this)           
            this.assertNotEmpty(this.ncsUnderTest.plant);            
            this.filter.setState(this.plantStateDistribution);            
            
            % initialize plant with deterministic state
            this.ncsUnderTest.initPlant(this.plantState);
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            
            % perform a control cycle without any data packets
            % so there is no input
            timestep = 1;
            this.ncsUnderTest.step(timestep, [], [], []);
            stats = this.ncsUnderTest.getStatistics().trueStates;
            trueState = stats(:, timestep + 1);
            
            expectedStageCosts = trueState' * this.Q * trueState;
            actualStageCosts = this.ncsUnderTest.getStageCosts(timestep);
            this.verifyEqual(actualStageCosts, expectedStageCosts);
         end
                
        
        %% testGetControlErrorNotInitialized
        function testGetControlErrorNotInitialized(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            % plant was set, but not initialized
            % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getControlError(1), 0);
            this.verifyEqual(this.ncsUnderTest.getControlError(10), 0);
        end
        
        %% testGetControlError
        function testGetControlError(this)
            this.assertNotEmpty(this.ncsUnderTest.plant);            
            this.filter.setState(this.plantStateDistribution);
            % initialize plant with deterministic state
            this.ncsUnderTest.initPlant(this.plantState);
            
            % and ensure that statistics is recorded
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            this.assertNotEmpty(this.ncsUnderTest.getStatistics());
            this.assertNotEmpty(this.ncsUnderTest.getStatistics().trueStates);
            this.assertNotEmpty(this.ncsUnderTest.getStatistics().controllerStates);
            
            % perform three control cycles without any data packets
            % so there is no input
            timestep = 1;
            for k=0:10
                this.ncsUnderTest.step(timestep+k, [], [], []);
            end            
            trueStates = this.ncsUnderTest.getStatistics().trueStates;
            controllerStates = this.ncsUnderTest.getStatistics().controllerStates;
            
            % compute "instantaneous error" at times 1, 2, 3
            %k=1
            expectedError = norm(trueStates(:, 2));
            expectedEstimatedError = norm(controllerStates(:, 2));
            [actualError, estimatedError] = this.ncsUnderTest.getControlError(1);
            this.verifyEqual(actualError, expectedError, 'AbsTol', 1e-8);
            this.verifyEqual(estimatedError, expectedEstimatedError, 'AbsTol', 1e-8);
            %k=2
            expectedError = norm(trueStates(:, 2)) + norm(trueStates(:, 3));
            expectedEstimatedError = norm(controllerStates(:, 2)) + norm(controllerStates(:, 3));
            [actualError, estimatedError] = this.ncsUnderTest.getControlError(2);
            this.verifyEqual(actualError, expectedError, 'AbsTol', 1e-8);
            this.verifyEqual(estimatedError, expectedEstimatedError, 'AbsTol', 1e-8);
            %k=3
            expectedError = norm(trueStates(:, 2)) + norm(trueStates(:, 3)) + norm(trueStates(:, 4));
            expectedEstimatedError = norm(controllerStates(:, 2)) + norm(controllerStates(:, 3)) + norm(controllerStates(:, 4));
            [actualError, estimatedError] = this.ncsUnderTest.getControlError(3);
            this.verifyEqual(actualError, expectedError, 'AbsTol', 1e-8);
            this.verifyEqual(estimatedError, expectedEstimatedError, 'AbsTol', 1e-8);
            
            %k=11
            expectedError = sum(sqrt(sum((trueStates(:, 3:12) .^ 2))));
            expectedEstimatedError = sum(sqrt(sum((controllerStates(:, 3:12) .^ 2))));
            [actualError, estimatedError] = this.ncsUnderTest.getControlError(11);
            this.verifyEqual(actualError, expectedError, 'AbsTol', 1e-8);
            this.verifyEqual(estimatedError, expectedEstimatedError, 'AbsTol', 1e-8);
        end
        
        %% testStepNoPackets
        function testStepNoPackets(this)
            import matlab.unittest.constraints.IsScalar;
                       
            this.assertNotEmpty(this.ncsUnderTest.plant); 
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.ncsUnderTest.initPlant(this.zeroPlantState);
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            this.filter.setState(this.zeroPlantStateDistribution);
                        
            timestep = 1;
            [caPacket, scPacket, controllerAck] = this.ncsUnderTest.step(timestep, [], [], []);
            
            this.verifyClass(caPacket, ?DataPacket);
            this.verifyClass(scPacket, ?DataPacket);
            
            inputSequence = caPacket.payload;
            measurement = scPacket.payload;
            
            this.verifyEmpty(controllerAck); % as there are no packets received
            this.verifySize(inputSequence, [this.dimU, this.controlSeqLength]);
            % we can only verify that the measurement is a scalar
            this.verifyThat(measurement, IsScalar);
            this.verifySize(measurement, [this.dimY, 1]);
            % linear controller, hence first element of sequence should be
            % zero
            % likewise, as A=I, and controller does not use noise w_k
            % all future states should be zero, and thus the remainder of
            % the sequence
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            
            % check timestep, source (id=3) and destination (id=2) of
            % scPacket packet
            this.verifyEqual(scPacket.timeStamp, timestep)
            this.verifyEqual(scPacket.sourceAddress, 3)
            this.verifyEqual(scPacket.destinationAddress, 2)
            
            % check timestep, source (id=2) and destination (id=1) of
            % caPacket packet
            this.verifyEqual(caPacket.timeStamp, timestep)
            this.verifyEqual(caPacket.sourceAddress, 2)
            this.verifyEqual(caPacket.destinationAddress, 1)
            
            stats = this.ncsUnderTest.getStatistics();
            recordedState = stats.trueStates(:, timestep + 1);
            recordedControllerState = stats.controllerStates(:, timestep + 1);
            recordedMode = stats.trueModes(timestep + 1);
            recordedInput = stats.appliedInputs(:, timestep);
            recordedNumUsedMeasurements = stats.numUsedMeasurements(timestep);
            recordedNumDiscardedMeasurements = stats.numDiscardedMeasurements(timestep);
            
            this.verifyEqual(recordedInput, this.actuator.defaultInput);
            this.verifyEqual(recordedState, this.zeroPlantState);
            this.verifyEqual(recordedControllerState, this.zeroPlantState);
            this.verifyEqual(recordedMode, this.controlSeqLength + 1);
            this.verifyEqual(recordedNumUsedMeasurements, 0);
            this.verifyEqual(recordedNumDiscardedMeasurements, 0);
        end
    end
    
end

