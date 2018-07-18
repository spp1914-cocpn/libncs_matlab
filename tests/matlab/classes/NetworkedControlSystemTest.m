classdef NetworkedControlSystemTest < matlab.unittest.TestCase
    % Test cases for NetworkedControlSystem.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (Constant, Access = private)
        ncsDefaultName = 'NCS';
        ncsDefaultSamplingInterval = 0.1;
        ncsDefaultNetworkType = NetworkType.UdpLikeWithAcks;
    end
    
    properties (Access = private)
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
            
            this.A = eye(this.dimX);
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
                this.filterPlantModel, this.sensor, zeros(this.dimU, 1));
            
            this.ncsUnderTest = NetworkedControlSystem(this.ncsName, this.ncsSamplingInterval, this.ncsNetworkType);
        end
    end
    
    methods (Test)
        %% testNetworkedControlSystemInvalidSamplingInterval
        function testNetworkedControlSystemInvalidSamplingInterval(this)
            expectedErrId = 'NetworkedControlSystem:InvalidSamplingInterval';
            
            invalidSamplingInterval = this; % not a scalar
            this.verifyError(@() NetworkedControlSystem(this.ncsName, invalidSamplingInterval), expectedErrId);
            
            invalidSamplingInterval = 0; % scalar value, but not positive
            this.verifyError(@() NetworkedControlSystem(this.ncsName, invalidSamplingInterval), expectedErrId);
            
            invalidSamplingInterval = 1i; % scalar value, but not a real value
            this.verifyError(@() NetworkedControlSystem(this.ncsName, invalidSamplingInterval), expectedErrId);
        end
        
        %% testNetworkedControlSystemNoArguments
        function testNetworkedControlSystemNoArguments(this)
            ncs = NetworkedControlSystem();
            
            this.verifyEqual(ncs.name, NetworkedControlSystemTest.ncsDefaultName);
            this.verifyEqual(ncs.samplingInterval, NetworkedControlSystemTest.ncsDefaultSamplingInterval);
            this.verifyEqual(ncs.networkType, NetworkedControlSystemTest.ncsDefaultNetworkType);
            
            this.verifyEmpty(ncs.sensor);
            this.verifyEmpty(ncs.controller);
            this.verifyEmpty(ncs.plant);
        end
        
        %% testNetworkedControlSystemOneArgument
        function testNetworkedControlSystemOneArgument(this)
            ncs = NetworkedControlSystem(this.ncsName);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, NetworkedControlSystemTest.ncsDefaultSamplingInterval);
            this.verifyEqual(ncs.networkType, NetworkedControlSystemTest.ncsDefaultNetworkType);
            
            this.verifyEmpty(ncs.sensor);
            this.verifyEmpty(ncs.controller);
            this.verifyEmpty(ncs.plant);
        end
        
        %% testNetworkedControlSystemTwoArguments
        function testNetworkedControlSystemTwoArguments(this)
            ncs = NetworkedControlSystem(this.ncsName, this.ncsSamplingInterval);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.networkType, NetworkedControlSystemTest.ncsDefaultNetworkType);
            
            this.verifyEmpty(ncs.sensor);
            this.verifyEmpty(ncs.controller);
            this.verifyEmpty(ncs.plant);
        end
        
         %% testNetworkedControlSystemThreeArguments
        function testNetworkedControlSystemThreeArguments(this)
            ncs = NetworkedControlSystem(this.ncsName, this.ncsSamplingInterval, this.ncsNetworkType);
            
            this.verifyEqual(ncs.name, this.ncsName);
            this.verifyEqual(ncs.samplingInterval, this.ncsSamplingInterval);
            this.verifyEqual(ncs.networkType, this.ncsNetworkType);
            
            this.verifyEmpty(ncs.sensor);
            this.verifyEmpty(ncs.controller);
            this.verifyEmpty(ncs.plant);
        end
        
        %% testInitPlantNoPlant
        function testInitPlantNoPlant(this)
            % initially, the plant is not set
            % assert this
            this.assertEmpty(this.ncsUnderTest.plant);
            expectedErrId = 'NetworkedControlSystem:CheckPlant';
            
            this.verifyError(@() this.ncsUnderTest.initPlant(this.plantState), expectedErrId);
        end
        
        %% testInitPlantInvalidState
        function testInitPlantInvalidState(this)
            % first, we need to set the plant
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.ncsUnderTest.controller = this.controllerSubsystem;
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
        
        %% testInitStatisticsRecordingNoPlant
        function testInitStatisticsRecordingNoPlant(this)
            % initially, the plant is not set
            % assert this
            this.assertEmpty(this.ncsUnderTest.plant);
            expectedErrId = 'NetworkedControlSystem:CheckPlant';
            
            this.verifyError(@() this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps), expectedErrId);
        end
                     
              
        %% testInitStatisticsRecordingInvalidMaxLoopSteps
        function testInitStatisticsRecordingInvalidMaxLoopSteps(this)
            % set plant and actuator
            this.ncsUnderTest.plant = this.plantSubsystem;
            
            this.assertNotEmpty(this.ncsUnderTest.plant);
             
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
            % set plant and controller
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.ncsUnderTest.controller = this.controllerSubsystem;
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
        
        %% testGetStatisticsNotInitialized
        function testGetStatisticsNotInitialized(this)
            % initially, the statistics property should be the empty matrix
            
            this.verifyEmpty(this.ncsUnderTest.getStatistics());
        end
        
        %% testGetStatistics
        function testGetStatistics(this)
            % set plant and controller and initialize the
            % recording of the statistics
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.ncsUnderTest.controller = this.controllerSubsystem;
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
        
        %% testGetStageCostsNoPlant
        function testGetStageCostsNoPlant(this)
            % initially, the plant is not set
            % assert this
            this.assertEmpty(this.ncsUnderTest.plant);
            expectedErrId = 'NetworkedControlSystem:CheckPlant';
            
            this.verifyError(@() this.ncsUnderTest.getStageCosts(1), expectedErrId);
        end
        
         %% testGetStageCostsNotInitialized
        function testGetStageCostsNotInitialized(this)
            this.ncsUnderTest.controller = this.controllerSubsystem;
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.assertNotEmpty(this.ncsUnderTest.plant);
            % plant was set, but not initialized
            % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getStageCosts(1), 0);
            this.verifyEqual(this.ncsUnderTest.getStageCosts(10), 0);
        end
        
        %% testGetStageCosts
        function testGetStageCosts(this)
            % set and initalize plant, controller, sensor
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.ncsUnderTest.controller = this.controllerSubsystem;
            this.filter.setState(this.plantStateDistribution);
            this.ncsUnderTest.sensor = this.sensorSubsystem;
            
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
        
        %% testGetQualityOfControlNoPlant
        function testGetQualityOfControlNoPlant(this)
            % initially, the plant is not set
            % assert this
            this.assertEmpty(this.ncsUnderTest.plant);
            expectedErrId = 'NetworkedControlSystem:CheckPlant';
            
            this.verifyError(@() this.ncsUnderTest.getQualityOfControl(1), expectedErrId);
        end
        
        %% testGetQualityOfControlNotInitialized
        function testGetQualityOfControlNotInitialized(this)
            this.ncsUnderTest.controller = this.controllerSubsystem;
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.assertNotEmpty(this.ncsUnderTest.plant);
            % plant was set, but not initialized
             % we expect zero to be returned independent of timestep
            
            this.verifyEqual(this.ncsUnderTest.getQualityOfControl(1), 0);
            this.verifyEqual(this.ncsUnderTest.getQualityOfControl(10), 0);
        end
        
        %% testGetQualityOfControl
        function testGetQualityOfControl(this)
            % set and initalize plant, controller
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.assertNotEmpty(this.ncsUnderTest.plant);
            this.ncsUnderTest.controller = this.controllerSubsystem;
            % initialize plant with deterministic state
            this.ncsUnderTest.initPlant(this.plantState);
            % plant state is [1 1 ... 1], so the norm is just sqrt(dimX)
            expectedQoc = sqrt(this.dimX);
            
            actualQoc = this.ncsUnderTest.getQualityOfControl(1);
             
            this.verifyEqual(actualQoc, expectedQoc);
            
            % now init the plant with a Gaussian state
            this.ncsUnderTest.initPlant(this.plantStateDistribution);
            % we can only verify that the resulting QoC value is nonnegative
            this.verifyGreaterThanOrEqual(this.ncsUnderTest.getQualityOfControl(1), 0);
        end
        
        %% testStepNoPackets
        function testStepNoPackets(this)
             import matlab.unittest.constraints.IsScalar;
            
           % set and initalize plant
            this.ncsUnderTest.plant = this.plantSubsystem;
            this.ncsUnderTest.controller = this.controllerSubsystem;
            
            this.assertNotEmpty(this.ncsUnderTest.plant); 
            this.assertNotEmpty(this.ncsUnderTest.controller);
            
            this.ncsUnderTest.initPlant(this.zeroPlantState);
            this.ncsUnderTest.initStatisticsRecording(this.maxLoopSteps);
            this.filter.setState(this.zeroPlantStateDistribution);
            
            this.ncsUnderTest.sensor = this.sensorSubsystem;
            
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

