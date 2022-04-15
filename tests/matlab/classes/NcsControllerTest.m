classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsControllerTest < matlab.unittest.TestCase
    % Test cases for NcsController.
    
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
    
    properties (Access = private)
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
        
        maxMeasDelay;
        controlSeqLength;
        plantStateOrigin;
        
        defaultInput;
        delayProbs;
        modeTransitionMatrix;
        transmissionCosts;
        
        controller;
        eventBasedController;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.dimX = 3;
            this.dimU = 2;
            this.dimY = 1;
            this.controlSeqLength = 3;
            this.maxMeasDelay = 3;
            
            % (A, B) is a controllable pair 
            this.A = [0 1 0; 
                      1 0 1;
                      0 0 1];
            this.B = [1 0;
                      0 1;
                      1 1];
            this.C = [1 2 3];
            this.W = eye(this.dimX); % sys noise cov
            this.V = 0.1^2; % variance of the meas noise
            this.Q = 2 * eye(this.dimX);
            this.R = 0.5 * eye(this.dimU);
     
            this.plantStateOrigin = ones(this.dimX, 1);            
            
            this.defaultInput = ones(1, this.dimU);
            this.delayProbs = ones(this.controlSeqLength, 1) / this.controlSeqLength;
            this.modeTransitionMatrix = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(this.delayProbs, this.controlSeqLength + 1));
            
            this.transmissionCosts = 10;
            
            v_mean = []; % implies zero
            useMex = false;
            this.controller = InfiniteHorizonUdpLikeController(this.A, this.B, this.C, this.Q, this.R, ...
                this.modeTransitionMatrix, this.delayProbs, this.controlSeqLength, this.maxMeasDelay, this.W, this.V, v_mean, useMex);            
            % this controller requires (A,B) controllable
            this.eventBasedController = EventTriggeredInfiniteHorizonController(this.A, this.B, this.Q, this.R, ...
                this.delayProbs, this.controlSeqLength, this.transmissionCosts);
        end
    end
    
    methods (Test)
        %% testNcsController
        function testNcsController(this)
            ncsController = NcsController(this.controller, this.defaultInput);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEmpty(ncsController.plantStateOrigin);
            this.verifyFalse(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, 1); % 1 second by default
                        
            ncsController = NcsController(this.controller, this.defaultInput, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyFalse(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, ...
                NcsController.defaultControlErrorWindowSize); % 1 second by default
            this.verifyEqual(ncsController.controlErrorWindowBaseSampleRate, ...
                NcsController.defaultControlErrorWindowBaseSampleInterval); % 1/200 second by default
            
            this.verifyTrue(ncsController.isControllerStateAdmissible());
        end
        
        %% testNcsControllerEventBased
        function testNcsControllerEventBased(this)            
            ncsController = NcsController(this.eventBasedController, this.defaultInput, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.eventBasedController);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyTrue(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, ...
                NcsController.defaultControlErrorWindowSize); % 1 second by default
            this.verifyEqual(ncsController.controlErrorWindowBaseSampleRate, ...
                NcsController.defaultControlErrorWindowBaseSampleInterval); % 1/200 second by default
            
            this.verifyTrue(ncsController.isControllerStateAdmissible());
        end       
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)            
            plantState = ones(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            % check the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = ncsController.getStatistics();
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyEmpty(actualStatistics.numUsedMeasurements);
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyEmpty(actualStatistics.numDiscardedMeasurements);
            % initial controller state is recorded correctly
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
            this.verifyEqual(actualStatistics.controllerStates, plantState);            
            this.assertTrue(isfield(actualStatistics, 'times'));
            this.assertEqual(actualStatistics.times, 0);
        end
        
        %% testGetStatistics
        function testGetStatistics(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);            
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            % assert the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = ncsController.getStatistics();
            this.assertTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.assertEmpty(actualStatistics.numUsedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.assertEmpty(actualStatistics.numDiscardedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'controllerStates'));
            this.assertEqual(actualStatistics.controllerStates, zeros(this.dimX, 1));
                        
            this.assertTrue(isfield(actualStatistics, 'times'));
            this.assertEqual(actualStatistics.times, 0);
            
            % perform 10 controller steps, without any measurements or ACKs
            % for this test, we assume one time step = 1 second
            doneTimeSteps = 10;
            acPackets = [];
            scPackets = [];
            previousMode = [];
            for j=1:doneTimeSteps
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end
                        
            % now get the recorded data and check if properly created and
            % recorded
            actualStatistics = ncsController.getStatistics();
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyEqual(actualStatistics.numUsedMeasurements, zeros(1, doneTimeSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyEqual(actualStatistics.numDiscardedMeasurements, zeros(1, doneTimeSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
            % sanity check: all values are filled, no nans, first is zero
            this.verifySize(actualStatistics.controllerStates, [this.dimX, doneTimeSteps + 1]);
            this.verifyEqual(actualStatistics.controllerStates(:, 1), zeros(this.dimX,1));
            this.verifyNotEqual(actualStatistics.controllerStates, nan(this.dimX, doneTimeSteps + 1));
            
            this.verifyTrue(isfield(actualStatistics, 'times'));
            % sanity check: all values are filled, no nans, first is zero, column vector
            this.verifySize(actualStatistics.times, [doneTimeSteps + 1 1]);            
            this.verifyEqual(actualStatistics.times, (0:doneTimeSteps)'); % one time step = 1 second
         end
        
        %% testGetStatisticsForTimestepInvalidTimestep
        function testGetStatisticsForTimestepInvalidTimestep(this)
            expectedErrId = 'NcsController:GetStatisticsForTimestep:InvalidTimestep';
            
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);            
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            % assert the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = ncsController.getStatistics();
            this.assertTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.assertEmpty(actualStatistics.numUsedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.assertEmpty(actualStatistics.numDiscardedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'controllerStates'));
            this.assertEqual(actualStatistics.controllerStates, zeros(this.dimX, 1));
            
            this.assertTrue(isfield(actualStatistics, 'times'));
            this.assertEqual(actualStatistics.times, 0);
            
            % perform 5 controller steps, without any measurements or ACKs
            % for this test, we assume one time step = 1 second
            doneTimeSteps = 5;
            acPackets = [];
            scPackets = [];
            previousMode = [];
            for j=1:doneTimeSteps
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end
            
            % now try to access data recorded at time 6
            this.verifyError(@() ncsController.getStatisticsForTimestep(doneTimeSteps + 1), expectedErrId);
        end
        
        %% testGetStatisticsForTimestep
        function testGetStatisticsForTimestep(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);            
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            % assert the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = ncsController.getStatistics();
            this.assertTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.assertEmpty(actualStatistics.numUsedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.assertEmpty(actualStatistics.numDiscardedMeasurements);
            
            this.assertTrue(isfield(actualStatistics, 'controllerStates'));
            this.assertEqual(actualStatistics.controllerStates, zeros(this.dimX, 1));
            
            this.assertTrue(isfield(actualStatistics, 'times'));
            this.assertEqual(actualStatistics.times, 0);
            
            % perform 5 controller steps, without any measurements or ACKs
            % for this test, we assume one time step = 1 second
            doneTimeSteps = 5;
            acPackets = [];
            scPackets = [];
            previousMode = [];
            for j=1:doneTimeSteps
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end
            
            % now obtain the statistics for time step 4
            timestep = 4;
            % call with four out params
            [numUsedMeasurements1, numDiscardedMeasurements1, controllerState1, simTimeSec] = ncsController.getStatisticsForTimestep(timestep);
            this.verifyEqual(numUsedMeasurements1, 0);
            this.verifyEqual(numDiscardedMeasurements1, 0);
            this.verifyEqual(simTimeSec, timestep);  % for this test, we assume one time step = 1 second
            
            % call with three out params
            [numUsedMeasurements2, numDiscardedMeasurements2, controllerState2] = ncsController.getStatisticsForTimestep(timestep);
            this.verifyEqual(numUsedMeasurements2, 0);
            this.verifyEqual(numDiscardedMeasurements2, 0);
            this.verifyEqual(controllerState1, controllerState2); % returnes state should be the same
        end
%%
%%
        %% testStepNoPacketsZeroState
        function testStepNoPacketsZeroState(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            timestep = 1;
            previousMode = [];
            
            % 1 time step = 1 second
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: controller is linear and
            % state is zero, so inputs should be zero
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
            
            this.verifyEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 0);
            this.verifyEqual(numDiscardedMeas, 0);
        end
        
        %% testStepZeroStateMeasDiscarded
        function testStepZeroStateMeasDiscarded(this)
            import matlab.unittest.fixtures.SuppressedWarningsFixture
            
            % disable the warning that will occur
            this.applyFixture(...
                SuppressedWarningsFixture('InfiniteHorizonUdpLikeController:GetApplicableMeasurements:IgnoringMeasurementsDelayTooLarge'));
            
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            previousMode = [];
            
            % 1 time step = 1 second
            % perform 10 steps, without measurement and acks, so that
            % statistics are recorded properly
            for j=1:10
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end
            % reset the state to zero            
            this.controller.setControllerPlantState(controllerState);
            
            timestep = 11;            
            % assume a measurement that is too old
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay -1;
            measDelay = timestep - measTime;
            this.assertGreaterThan(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
            previousMode = [];
            
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
            
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: controller is linear and
            % state is zero, so inputs should be zero
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            this.verifyEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 0);
            this.verifyEqual(numDiscardedMeas, 1);          
        end
        
        %% testStepZeroStateMeasUsed
        function testStepZeroStateMeasUsed(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            previousMode = [];
            
            % 1 time step = 1 second
            % perform 20 steps, without measurement and acks, so that
            % statistics are recorded properly
            for j=1:20
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end

            % reset the state to zero            
            this.controller.setControllerPlantState(controllerState);
                        
            timestep = 21;
            % assume a measurement that is still applicable
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay;
            measDelay = timestep - measTime;
            this.assertEqual(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
            previousMode = [];
            
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
                
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: controller is linear and
            % state is zero, so inputs should be zero
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            this.verifyEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 1);
            this.verifyEqual(numDiscardedMeas, 0);
        end
        
        %% testComputeCosts
        function testComputeCosts(this)
            ncsController = NcsController(this.controller, this.defaultInput);
            
            horizonLength = 100;
            state = zeros(this.dimX, 1) + 42;
            input = zeros(this.dimU, 1) - 2;
            
            % assume constant state and input for simplicity
            stateTrajectory = repmat(state, 1, horizonLength + 1);
            inputTrajectory = repmat(input, 1, horizonLength);
            
            % we have an infinite-horizon controller in use, so average
            % costs are computed
            expectedCosts = (state' * this.Q * state + horizonLength * (state' * this.Q * state + input' * this.R * input)) / horizonLength;
            actualCosts = ncsController.computeCosts(stateTrajectory, inputTrajectory);
            
            this.verifyEqual(expectedCosts, actualCosts);
            
            % now consider the case when the origin is shifted
            ncsController = NcsController(this.controller, this.defaultInput, this.plantStateOrigin);
            
            horizonLength = 100;
            state = zeros(this.dimX, 1) + 42;
            input = zeros(this.dimU, 1) - 2;
            
            % assume constant state and input for simplicity
            stateTrajectory = repmat(state, 1, horizonLength + 1);
            inputTrajectory = repmat(input, 1, horizonLength);
            
            % we have an infinite-horizon controller in use, so average
            % costs are computed
            stateDiff = state - this.plantStateOrigin;
            expectedCosts = (stateDiff' * this.Q * stateDiff + horizonLength * (stateDiff' * this.Q * stateDiff + input' * this.R * input)) / horizonLength;
            actualCosts = ncsController.computeCosts(stateTrajectory, inputTrajectory);
            
            this.verifyEqual(expectedCosts, actualCosts);
        end
        
        %% testGetCurrentStageCosts
        function testGetCurrentStageCosts(this)
            ncsController = NcsController(this.controller, this.defaultInput);
            
            state = zeros(this.dimX, 1) + 42;
            input = zeros(this.dimU, 1) - 2;
            timestep = 1;
            
            expectedStageCosts = state' * this.Q * state + input' * this.R * input;
            actualStageCosts = ncsController.getCurrentStageCosts(state, input, timestep);
            
            this.verifyEqual(actualStageCosts, expectedStageCosts);
            
            % now consider the case when the origin is shifted
            ncsController = NcsController(this.controller, this.defaultInput, this.plantStateOrigin);
            
            expectedStageCosts = (state'-this.plantStateOrigin') * this.Q * (state-this.plantStateOrigin) ...
                + input' * this.R * input;
            actualStageCosts = ncsController.getCurrentStageCosts(state, input, timestep);
            
            this.verifyEqual(actualStageCosts, expectedStageCosts);
        end
        
%         %% testGetCurrentQualityOfControlSingleHorizon
%         function testGetCurrentQualityOfControlSingleHorizon(this)
%             ncsController = NcsController(this.controller);
%             
%             ncsController.initStatisticsRecording(1, this.dimX);
%             
%             timestep = 1;
%             states = zeros(this.dimX, 10) + 42;
%             
%             expectedQoC = sqrt(42^2 * this.dimX); % simply the norm
%             [~, actualQoC] = ncsController.getCurrentQualityOfControl(timestep, states);
%             
%             this.verifyEqual(actualQoC, expectedQoC);
%             
%             % now consider the case when the origin is shifted
%             ncsController = NcsController(this.controller, this.plantStateOrigin);
%             
%             expectedQoC = norm(zeros(this.dimX, 1) + 42 - this.plantStateOrigin); % simply the norm
%             [~, actualQoC] = ncsController.getCurrentQualityOfControl(timestep, states);
%             
%             this.verifyEqual(actualQoC, expectedQoC);            
%         end
%         
%         %% testGetCurrentQualityOfControl
%         function testGetCurrentQualityOfControl(this)
%             ncsController = NcsController(this.controller);          
%             
%             % now we use a longer horizon (N=10)
%             timestep = 10;
%             states = zeros(this.dimX, timestep) + 56;
%             
%             expectedQoC = sqrt(56^2 * this.dimX) * 10; % simply the norm
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(states, timestep);
%             
%             this.verifyEqual(expectedQoC, actualQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedQoC, 'AbsTol', 1e-8);
%             
%             % now consider the case when the origin is shifted
%             ncsController = NcsController(this.controller, this.plantStateOrigin);
%             
%             expectedQoC = sum(sqrt(sum((states - this.plantStateOrigin) .^ 2)));
%             expectedEstimatedQoC = sqrt(56^2 * this.dimX) * 10;
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(states, timestep);
%             
%             this.verifyEqual(actualQoC, expectedQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedEstimatedQoC, 'AbsTol', 1e-8);
%         end
%         
%         %% testGetCurrentQualityOfControlSetpointSingleHorizon
%         function testGetCurrentQualityOfControlSetpointSingleHorizon(this)
%             % now use a controller that supports tracking
%             trackingController = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
%             
%             setpoint = zeros(this.dimX, 1) + 42;
%             trackingController.changeSetPoint(setpoint);
%             ncsController = NcsController(trackingController);
%             
%             timestep = 1;
%             state = setpoint;
%             
%             expectedQoC = 0;
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(state, timestep, state);
%             
%             this.verifyEqual(actualQoC, expectedQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedQoC, 'AbsTol', 1e-8);
%             
%             % now consider the case when the origin is shifted
%             ncsController = NcsController(trackingController, this.plantStateOrigin);
%             
%             expectedQoC = norm(state - this.plantStateOrigin - setpoint); % simply the accumulated norm of error
%             expectedEstimatedQoC = norm(state - setpoint);
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(state, timestep, state);
%             
%             this.verifyEqual(actualQoC, expectedQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedEstimatedQoC, 'AbsTol', 1e-8);
%         end
%         
%          %% testGetCurrentQualityOfControlSetpoint
%         function testGetCurrentQualityOfControlSetpoint(this)
%             % now use a controller that supports tracking
%             trackingController = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
%             
%             setpoint = zeros(this.dimX, 1) + 42;
%             trackingController.changeSetPoint(setpoint);
%             ncsController = NcsController(trackingController);
%             
%             % now we use a longer horizon (N=10)
%             timestep = 10;
%             states = repmat(setpoint, 1, timestep);
%             
%             expectedQoC = 0;
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(states, timestep, states);
%             
%             this.verifyEqual(actualQoC, expectedQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedQoC, 'AbsTol', 1e-8);
%             
%             % now consider the case when the origin is shifted
%             ncsController = NcsController(trackingController, this.plantStateOrigin);
%   
%             expectedQoC = sum(sqrt(sum((states - this.plantStateOrigin - setpoint) .^ 2))); % simply the accumulated norm of error
%             expectedEstimatedQoC = 0;
%             [actualQoC, estimatedQoC] = ncsController.getCurrentQualityOfControl(states, timestep, states);
%             
%             this.verifyEqual(actualQoC, expectedQoC, 'AbsTol', 1e-8);
%             this.verifyEqual(estimatedQoC, expectedEstimatedQoC, 'AbsTol', 1e-8);
%         end
        
        %% testChangeSequenceLengthFalse
        function testChangeSequenceLengthFalse(this)
            newSeqLength = this.controlSeqLength + 1;
            
            % with the controller in use this is not possible
            ncsController = NcsController(this.controller, this.defaultInput);
            this.assertEqual(ncsController.controller.sequenceLength, this.controlSeqLength);           
            
            this.verifyFalse(ncsController.changeSequenceLength(newSeqLength));
            this.verifyEqual(this.controller.sequenceLength, this.controlSeqLength);           
        end

        %% testChangeSequenceLengthTrue
        function testChangeSequenceLengthTrue(this)
            newSeqLength = this.controlSeqLength + 1;
                       
            % use a controller that supports changing the sequence
            % length at runtime
            contr = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            ncsController = NcsController(contr, this.defaultInput);            
            this.assertEqual(ncsController.controller.sequenceLength, this.controlSeqLength);
            
            this.verifyTrue(ncsController.changeSequenceLength(newSeqLength));
            this.verifyEqual(contr.sequenceLength, newSeqLength);
        end
%%
%%
        %% testChangeCaDelayProbsFalse
        function testChangeCaDelayProbsFalse(this)
            mixinClass = ?CaDelayProbsChangeable;
            this.assertFalse(ismember(mixinClass.Name, superclasses(this.controller)));
            % this is currently generally not possible for most controllers
            % in use
            
            ncsController = NcsController(this.controller, this.defaultInput);
            
            newDelayProbs = [this.delayProbs(:); 0];
            this.verifyFalse(ncsController.changeCaDelayProbs(newDelayProbs));                     
        end
        
        %% testChangeCaDelayProbsTrue
        function testChangeCaDelayProbsTrue(this)
            % for the IMM-based controller, this operation is supported
            horizonlength = 5;
            immBasedController = IMMBasedRecedingHorizonController(this.A, this.B, this.C, this.Q, this.R, ...
                this.modeTransitionMatrix, this.controlSeqLength, this.maxMeasDelay, this.W, ...
                Gaussian(zeros(this.dimY, 1), this.V), horizonlength, zeros(this.dimX, 1), eye(this.dimX));
            
            mixinClass = ?CaDelayProbsChangeable;
            this.assertTrue(ismember(mixinClass.Name, superclasses(immBasedController)));
            
            ncsController = NcsController(immBasedController, this.defaultInput);
            newDelayProbs = [this.delayProbs(:); 0];
            
            this.verifyTrue(ncsController.changeCaDelayProbs(newDelayProbs));
        end
        
        %% testChangeCaDelayProbsInvalidProbs
        function testChangeCaDelayProbsInvalidProbs(this)
            expectedErrId = 'Validator:ValidateDiscreteProbabilityDistribution:InvalidProbs';
            
            % for the IMM-based controller, this operation is supported
            horizonlength = 5;
            immBasedController = IMMBasedRecedingHorizonController(this.A, this.B, this.C, this.Q, this.R, ...
                this.modeTransitionMatrix, this.controlSeqLength, this.maxMeasDelay, this.W, ...
                Gaussian(zeros(this.dimY, 1), this.V), horizonlength, zeros(this.dimX, 1), eye(this.dimX));
            
            mixinClass = ?CaDelayProbsChangeable;
            this.assertTrue(ismember(mixinClass.Name, superclasses(immBasedController)));
           
            ncsController = NcsController(immBasedController, this.defaultInput);
            % call should error in case of invalid distribution
            invalidDistribution = eye(4);
            this.verifyError(@() ncsController.changeCaDelayProbs(invalidDistribution), expectedErrId);
        end
%%
%%
        %% testChangeScDelayProbsFalse
        function testChangeScDelayProbsFalse(this)
            mixinClass = ?ScDelayProbsChangeable;
            this.assertFalse(ismember(mixinClass.Name, superclasses(this.controller)));
            % this is currently generally not possible for most controllers
            % in use
            
            ncsController = NcsController(this.controller, this.defaultInput);
            
            newDelayProbs = [this.delayProbs(:); 0];
            this.verifyFalse(ncsController.changeScDelayProbs(newDelayProbs));                     
        end
        
        %% testChangeScDelayProbsTrue
        function testChangeScDelayProbsTrue(this)
            % use controller that supports this
            horizonlength = 5;
            mpc = RecedingHorizonUdpLikeController(this.A, this.B, this.C, this.Q, this.R, this.modeTransitionMatrix, this.delayProbs, ...
                this.controlSeqLength, this.maxMeasDelay, this.W, this.V, horizonlength,  zeros(this.dimX, 1), eye(this.dimX), false);
            
            mixinClass = ?ScDelayProbsChangeable;
            this.assertTrue(ismember(mixinClass.Name, superclasses(mpc)));
            
            ncsController = NcsController(mpc, this.defaultInput);
            newDelayProbs = [this.delayProbs(:); 0];
            
            this.verifyTrue(ncsController.changeScDelayProbs(newDelayProbs));
        end
        
        %% testChangeScDelayProbsInvalidProbs
        function testChangeScDelayProbsInvalidProbs(this)
            expectedErrId = 'Validator:ValidateDiscreteProbabilityDistribution:InvalidProbs';
            
            % use controller that supports this
            horizonlength = 5;
            mpc = RecedingHorizonUdpLikeController(this.A, this.B, this.C, this.Q, this.R, this.modeTransitionMatrix, this.delayProbs, ...
                this.controlSeqLength, this.maxMeasDelay, this.W, this.V, horizonlength,  zeros(this.dimX, 1), eye(this.dimX), false);
            
            mixinClass = ?ScDelayProbsChangeable;
            this.assertTrue(ismember(mixinClass.Name, superclasses(mpc)));
           
            ncsController = NcsController(mpc, this.defaultInput);
            % call should error in case of invalid distribution
            invalidDistribution = eye(4);
            this.verifyError(@() ncsController.changeScDelayProbs(invalidDistribution), expectedErrId);
        end
%%
%%
        %% testChangeModelParametersFalse
        function testChangeModelParametersFalse(this)
            mixinClass = ?ModelParamsChangeable;
            this.assertFalse(ismember(mixinClass.Name, superclasses(this.controller)));
            
            newA = this.A * 2;
            newB = this.B * 2;
            newW = this.W * 2;
            
            ncsController = NcsController(this.controller, this.defaultInput);
            this.verifyFalse(ncsController.changeModelParameters(newA, newB, newW));
        end
        
        %% testChangeModelParametersTrue
        function testChangeModelParametersTrue(this)
            mixinClass = ?ModelParamsChangeable;
            
            horizonlength = 5;
            immBasedController = IMMBasedRecedingHorizonController(this.A, this.B, this.C, this.Q, this.R, ...
                this.modeTransitionMatrix, this.controlSeqLength, this.maxMeasDelay, this.W, ...
                Gaussian(zeros(this.dimY, 1), this.V), horizonlength, zeros(this.dimX, 1), eye(this.dimX));
            
            this.assertTrue(ismember(mixinClass.Name, superclasses(immBasedController)));
            
            newA = this.A * 2;
            newB = this.B * 2;
            newW = this.W * 2;
            
            ncsController = NcsController(immBasedController, this.defaultInput);
            this.verifyTrue(ncsController.changeModelParameters(newA, newB, newW));
        end
    end
end

