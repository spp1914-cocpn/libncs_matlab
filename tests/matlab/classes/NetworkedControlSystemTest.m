classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NetworkedControlSystemTest < matlab.unittest.TestCase
    % Test cases for NetworkedControlSystem.
    
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
    
    properties (Access = private)
        ncsDefaultSamplingInterval = 0.1;
        ncsDefaultPlantSamplingInterval = 0.001;
        ncsDefaultNetworkType = NetworkType.UdpLikeWithAcks;
        
        ncsName;
        ncsSamplingInterval;
        plantSamplingInterval;
        ncsNetworkType;
        
        maxPlantSteps;
        maxControllerSteps;
        maxSimTime;
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
            this.plantSamplingInterval = this.ncsSamplingInterval; % assume same sampling rate for testing
            this.ncsNetworkType = NetworkType.TcpLike;
            
            this.maxPlantSteps = 100;
            this.maxControllerSteps = this.maxPlantSteps;
            this.maxSimTime = ConvertToPicoseconds(this.maxPlantSteps * this.plantSamplingInterval);
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
            
            this.filter = DelayedKF(this.maxMeasDelay, eye(3));
            %this.filter.setState(this.plantStateDistribution);
            this.filterPlantModel = LinearPlant(this.A, this.B, this.W);
            
            this.actuator = BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1));
            this.plantSubsystem = NcsPlant(this.plant, this.actuator);
            
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.controllerSubsystem = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4]');
            
            this.ncsUnderTest = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.plantSamplingInterval, this.ncsNetworkType);
        end
    end
    
    methods (Test)
        %% testNetworkedControlSystemNoOptionalArguments
        function testNetworkedControlSystemNoOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, this.ncsName);
            
            this.verifyEqual(ncs.samplingInterval, this.ncsDefaultSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.ncsDefaultPlantSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            this.verifyTrue(ncs.isPlantStateAdmissible());
            this.verifyTrue(ncs.isControllerStateAdmissible());
        end
        
        %% testNetworkedControlSystemOneOptionalArgument
        function testNetworkedControlSystemOneOptionalArgument(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, this.ncsName, this.ncsSamplingInterval);
                        
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.ncsDefaultPlantSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            this.verifyTrue(ncs.isPlantStateAdmissible());
            this.verifyTrue(ncs.isControllerStateAdmissible());
        end
        
        %% testNetworkedControlSystemTwoOptionalArguments
        function testNetworkedControlSystemTwoOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.plantSamplingInterval);
            
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.plantSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsDefaultNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            this.verifyTrue(ncs.isPlantStateAdmissible());
            this.verifyTrue(ncs.isControllerStateAdmissible());
        end
        
        %% testNetworkedControlSystemThreeOptionalArguments
        function testNetworkedControlSystemThreeOptionalArguments(this)
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.plantSamplingInterval, this.ncsNetworkType);
            
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.plantSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            % pass network type as integer
            type = NetworkType.TcpLike;
            ncs = NetworkedControlSystem(this.controllerSubsystem, this.plantSubsystem, this.sensorSubsystem, ...
                this.ncsName, this.ncsSamplingInterval, this.plantSamplingInterval, int64(type));
            
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.plantSamplingInterval);
            this.verifyEqual(ncs.networkType, type);     

            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.sensor, this.sensorSubsystem);
            this.verifyEqual(ncs.controller, this.controllerSubsystem);
            this.verifyEqual(ncs.plant, this.plantSubsystem);
            
            this.verifyTrue(ncs.isPlantStateAdmissible());
            this.verifyTrue(ncs.isControllerStateAdmissible());
        end
%%
%%
        %% testEvaluateRateQualityCharacteristicsInvalidTranslator
        function testEvaluateRateQualityCharacteristicsInvalidTranslator(this)
            expectedErrId = 'NetworkedControlSystem:CheckTranslator';
            
            targetQoc = 1;
            this.verifyError(@() this.ncsUnderTest.evaluateRateQualityCharacteristics(targetQoc), expectedErrId);
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
         
            desiredQoc = 1;
            
            [dataRate, rateChange] = this.ncsUnderTest.evaluateRateQualityCharacteristics(desiredQoc);
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
    
            targetRate = 50; % packets per second
            this.verifyError(@() this.ncsUnderTest.evaluateQualityRateCharacteristics(targetRate), expectedErrId);
        end
        
        %% testEvaluateQualityRateCharacteristics
        function testEvaluateQualityRateCharacteristics(this)
            import matlab.unittest.constraints.IsScalar
            import matlab.unittest.constraints.IsGreaterThanOrEqualTo            
            
            qocRateCurve = cfit(fittype('a*x^2'), 1);
            controlErrorQocCurve = cfit(fittype('a*x^3'), 1.5);
            translator = NcsTranslator(qocRateCurve, controlErrorQocCurve, 1 / this.ncsUnderTest.samplingInterval);
            
            this.ncsUnderTest.attachTranslator(translator);
   
            targetRate = 50; % packets per second
            
            qoc = this.ncsUnderTest.evaluateQualityRateCharacteristics(targetRate);
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
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState, this.maxSimTime), expectedErrId);
            
            invalidState = ones(this.dimX +1, 1); % vector, but incorrect number of elements
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState, this.maxSimTime), expectedErrId);
            
            invalidState = ones(this.dimX, 1); % vector, but contains inf
            invalidState(end) = inf;
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState, this.maxSimTime), expectedErrId);
            
            invalidState = 'state'; % vector, but no numerical value
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState, this.maxSimTime), expectedErrId);
            
            invalidState = Gaussian(ones(this.dimX +1, 1), eye(this.dimX + 1)); % Distribution, but wrong dimension
            this.verifyError(@() this.ncsUnderTest.initPlant(invalidState, this.maxSimTime), expectedErrId);
        end
%%
%%        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.filter.setState(this.zeroPlantStateDistribution);
            this.ncsUnderTest.initPlant(this.plantState, this.maxSimTime);
           
            % should be a successful call
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
            
            % now perform all the steps
            for i=1:this.maxPlantSteps % equals max controller steps
                timestamp = ConvertToPicoseconds(i * this.plantSamplingInterval);
                this.ncsUnderTest.plantStep(timestamp);
                this.ncsUnderTest.step(timestamp, [], [], []);
            end
            
            % and obtain the data gathered during "simulation"
            % check if initial conditions were set correctly
            stats = this.ncsUnderTest.getStatistics();            
            
            this.verifyNotEmpty(stats);
            recordedIntialState = stats.trueStates(:, 1);
            recordedInitialMode = stats.trueModes(1);
            recordedInitialInput = stats.appliedInputs(:, 1);             
            recordedInitialControllerState = stats.controllerStates(:, 1);
            recordedInitialControllerTime = stats.times(1);            
            
            this.verifyEqual(recordedIntialState, this.plantState);
            this.verifyFalse(isnan(recordedInitialMode));
            this.verifyEqual(sum(isnan(recordedInitialInput)), 0);
            this.verifyEqual(recordedInitialControllerState, this.zeroPlantState); % the expected filter/controller state
            this.verifyEqual(recordedInitialControllerTime, 0);
            
        end
                
        %% testGetStatistics
        function testGetStatistics(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.filter.setState(this.zeroPlantStateDistribution);
            this.ncsUnderTest.initPlant(this.plantState, this.maxSimTime);
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
            
            % now perform all the steps
            for i=1:this.maxPlantSteps % equals max controller steps
                timestamp = ConvertToPicoseconds(i * this.plantSamplingInterval);
                this.ncsUnderTest.plantStep(timestamp);
                this.ncsUnderTest.step(timestamp, [], [], []);
            end
            
            % and obtain the data gathered during "simulation"
            actualStatistics = this.ncsUnderTest.getStatistics();
            
            this.verifyTrue(isstruct(actualStatistics));
            
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
            this.verifyTrue(isfield(actualStatistics, 'times'));
%                              
            this.verifySize(actualStatistics.trueStates, [this.dimX this.maxPlantSteps+1]);
            this.verifySize(actualStatistics.trueModes, [1 this.maxControllerSteps]);
            this.verifySize(actualStatistics.appliedInputs, [this.dimU this.maxPlantSteps]);
            this.verifySize(actualStatistics.numUsedMeasurements, [1 this.maxControllerSteps]);
            this.verifySize(actualStatistics.numDiscardedMeasurements, [1 this.maxControllerSteps]);
            this.verifySize(actualStatistics.numDiscardedControlSequences, [1 this.maxControllerSteps]);
            this.verifySize(actualStatistics.controllerStates, [this.dimX this.maxControllerSteps+1]);
            this.verifySize(actualStatistics.times, [this.maxControllerSteps+1 1]); % a column vector
        end
        
        %% testGetStatisticsNotAllSteps
        function testGetStatisticsNotAllSteps(this)
            newMaxPlantSteps = 10;
            newMaxControllerSteps = 10;
            this.assertLessThan(newMaxPlantSteps, this.maxPlantSteps);
            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.filter.setState(this.zeroPlantStateDistribution);
            this.ncsUnderTest.initPlant(this.plantState, this.maxSimTime);
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
                                 
            % now perform the steps
            for i=1:newMaxPlantSteps % equals new max controller steps
                timestamp = ConvertToPicoseconds(i * this.plantSamplingInterval);
                this.ncsUnderTest.plantStep(timestamp);
                this.ncsUnderTest.step(timestamp, [], [], []);
            end
            
            % and obtain the data gathered during "simulation"
            actualStatistics = this.ncsUnderTest.getStatistics();
            
            this.verifyTrue(isstruct(actualStatistics));
            
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
            this.verifyTrue(isfield(actualStatistics, 'times'));
%                              
            this.verifySize(actualStatistics.trueStates, [this.dimX, newMaxPlantSteps+1]);
            this.verifySize(actualStatistics.trueModes, [1 newMaxControllerSteps]);
            this.verifySize(actualStatistics.appliedInputs, [this.dimU newMaxPlantSteps]);
            this.verifySize(actualStatistics.numUsedMeasurements, [1 newMaxControllerSteps]);
            this.verifySize(actualStatistics.numDiscardedMeasurements, [1 newMaxControllerSteps]);
            this.verifySize(actualStatistics.numDiscardedControlSequences, [1 newMaxControllerSteps]);
            this.verifySize(actualStatistics.controllerStates, [this.dimX newMaxControllerSteps+1]);
            this.verifySize(actualStatistics.times, [newMaxControllerSteps+1 1]); % a column vector
        end
        
        %% testGetCurrentStageCostsNotInitialized
        function testGetCurrentStageCostsNotInitialized(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            % plant and controller were set, but not initialized or used
            % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getCurrentStageCosts(), 0);
            
            % controller was not invoked
            this.verifyEqual(this.ncsUnderTest.getCurrentStageCosts(), 0);
        end
        
        %% testGetCurrentStageCosts
        function testGetCurrentStageCosts(this)           
            this.assertNotEmpty(this.ncsUnderTest.plant);            
            this.assertNotEmpty(this.ncsUnderTest.controller);
            this.filter.setState(this.plantStateDistribution);        
            
            % initialize plant with deterministic state
            rng(42); % for reproducability, init() draws the noise samples
            this.ncsUnderTest.initPlant(this.plantState, this.maxSimTime);
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
            
            % perform a control cycle without any data packets
            % so there is no input
            plantTimestamp = ConvertToPicoseconds(this.plantSamplingInterval); % in pico seconds
            controllerTimestamp = ConvertToPicoseconds(this.ncsSamplingInterval); % in pico seconds
         
            this.ncsUnderTest.plantStep(plantTimestamp);            
            this.ncsUnderTest.step(controllerTimestamp, [], [], []);

            rng(42); % get the same noise sample
            trueState = this.A * this.plantState + this.plant.noise.drawRndSamples(1);
            
            expectedStageCosts = trueState' * this.Q * trueState;
            actualStageCosts = this.ncsUnderTest.getCurrentStageCosts();
            this.verifyEqual(actualStageCosts, expectedStageCosts);
        end
                
        %% testComputeTotalControlCosts
        function testComputeTotalControlCosts(this)
            this.assertNotEmpty(this.ncsUnderTest.plant);            
            this.assertNotEmpty(this.ncsUnderTest.controller);
            this.filter.setState(this.plantStateDistribution);            
            
            % initialize plant with deterministic state
            rng(42); % for reproducability, init() draws the noise samples
            this.ncsUnderTest.initPlant(this.plantState, this.maxSimTime);
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
            
            % perform a control cycle without any data packets
            % so there is no input
            plantTimestamp = ConvertToPicoseconds(this.plantSamplingInterval); % in pico seconds
            controllerTimestamp = ConvertToPicoseconds(this.ncsSamplingInterval); % in pico seconds            
            
            this.ncsUnderTest.plantStep(plantTimestamp);            
            this.ncsUnderTest.step(controllerTimestamp, [], [], []);

            rng(42); % get the same noise sample
            trueState = this.A * this.plantState + this.plant.noise.drawRndSamples(1);
            % we have no inputs
            expectedTotalCost = this.plantState' * this.Q * this.plantState + trueState' * this.Q * trueState;
            actualTotalCost = this.ncsUnderTest.computeTotalControlCosts();
            this.verifyEqual(actualTotalCost, expectedTotalCost);
        end
        
        %% testGetCurrentControlErrorNotInitialized
        function testGetCurrentControlErrorNotInitialized(this)            
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.assertNotEmpty(this.ncsUnderTest.controller);
            % plant and controller were set, but not initialized or used
            % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getCurrentControlError(), 0);            
        end        
        
        %% testStepNoPackets
        function testStepNoPackets(this)
            import matlab.unittest.constraints.IsScalar;
                       
            this.assertNotEmpty(this.ncsUnderTest.plant); 
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.ncsUnderTest.initPlant(this.zeroPlantState, this.maxSimTime);           
            this.filter.setState(this.zeroPlantStateDistribution);            
            this.ncsUnderTest.initStatisticsRecording(this.maxSimTime);
            
            timestep = 1;
            controllerTimestamp = ConvertToPicoseconds(timestep * this.ncsSamplingInterval); % in pico seconds
            plantTimestamp = ConvertToPicoseconds(timestep * this.plantSamplingInterval); % in pico seconds
            this.ncsUnderTest.plantStep(plantTimestamp);
            
            [caPacket, scPacket, controllerAck] = this.ncsUnderTest.step(controllerTimestamp, [], [], []);
            
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
            recordedState = stats.trueStates(:, timestep);
            recordedControllerState = stats.controllerStates(:, timestep + 1);
            recordedMode = stats.trueModes(timestep);
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

