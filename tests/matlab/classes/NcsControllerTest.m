classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsControllerTest < matlab.unittest.TestCase
    % Test cases for NcsController.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2019  Florian Rosenthal <florian.rosenthal@kit.edu>
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
            this.transmissionCosts = 10;
            
            this.controller = InfiniteHorizonUdpLikeController(this.A, this.B, this.C, this.Q, this.R, ...
                this.delayProbs, this.delayProbs, this.controlSeqLength, this.maxMeasDelay, this.W, this.V);
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
            this.verifyEqual(ncsController.controlErrorWindowSize, 10);            
                        
            ncsController = NcsController(this.controller, this.defaultInput, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyFalse(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, 10);
        end
        
        %% testNcsControllerEventBased
        function testNcsControllerEventBased(this)
            invalidPlantStateOrigin = ones(this.dimX); % a matrix
            ncsController = NcsController(this.eventBasedController, this.defaultInput, invalidPlantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.eventBasedController);
            this.verifyEmpty(ncsController.plantStateOrigin);
            this.verifyTrue(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, 10);
            
            ncsController = NcsController(this.eventBasedController, this.defaultInput, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.eventBasedController);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyTrue(ncsController.isEventBased);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyEqual(ncsController.controlErrorWindowSize, 10);
        end
        
        %% testSetControlErrorWindowSize
        function testSetControlErrorWindowSize(this)
            ncsController = NcsController(this.controller, this.defaultInput);
            
            this.assertEqual(ncsController.controlErrorWindowSize, 10);
            expectedErrId = 'NcsController:SetControlErrorWindowSize:InvalidWindowSize';
                        
            invalidSize = -1; % negative
            errorThrown = false;
            try
                ncsController.controlErrorWindowSize = invalidSize;
            catch ex
                errorThrown = true;
                this.verifyEqual(ex.identifier, expectedErrId);
            end 
            if ~errorThrown
               this.verifyFail(); % failure 
            end
            
            invalidSize = this; % not a scalar
            errorThrown = false;
            try
                ncsController.controlErrorWindowSize = invalidSize;
            catch ex
                errorThrown = true;
                this.verifyEqual(ex.identifier, expectedErrId);
            end 
            if ~errorThrown
               this.verifyFail(); % failure 
            end
            
            invalidSize = 42.5; % not an integer
            errorThrown = false;
            try
                ncsController.controlErrorWindowSize = invalidSize;
            catch ex
                errorThrown = true;
                this.verifyEqual(ex.identifier, expectedErrId);
            end 
            if ~errorThrown
               this.verifyFail(); % failure 
            end
            
            % this call should succeed
            validSize = 42;
            ncsController.controlErrorWindowSize = validSize;
            this.verifyEqual(ncsController.controlErrorWindowSize, validSize);
        end
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)
            maxLoopSteps = 100;
            dimState = this.dimX;
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording(maxLoopSteps, dimState);
            
            % check the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = ncsController.statistics;
            this.verifyTrue(isfield(actualStatistics, 'numUsedMeasurements'));
            this.verifyEqual(actualStatistics.numUsedMeasurements, nan(1, maxLoopSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedMeasurements'));
            this.verifyEqual(actualStatistics.numDiscardedMeasurements, nan(1, maxLoopSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'controllerStates'));
            this.verifyEqual(actualStatistics.controllerStates, zeros(this.dimX, maxLoopSteps + 1));           
        end
        
        %% testStepNoPacketsZeroState
        function testStepNoPacketsZeroState(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            
            ncsController.initStatisticsRecording(30, this.dimX);
            
            acPackets = [];
            scPackets = [];
            timestep = 21;
            previousMode = [];
            
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
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
            
            numUsedMeas = ncsController.statistics.numUsedMeasurements(timestep);
            numDiscardedMeas = ncsController.statistics.numDiscardedMeasurements(timestep);
            actualControllerState = ncsController.statistics.controllerStates(:, timestep + 1);
            
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
                        
            timestep = 21;
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording(30, this.dimX);
            
            % assume a measurement that is too old
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay -1;
            measDelay = timestep - measTime;
            this.assertGreaterThan(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
            previousMode = [];
            
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
            numUsedMeas = ncsController.statistics.numUsedMeasurements(timestep);
            numDiscardedMeas = ncsController.statistics.numDiscardedMeasurements(timestep);
            actualControllerState = ncsController.statistics.controllerStates(:, timestep + 1);
            
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
            timestep = 21;
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.controller.setControllerPlantState(controllerState);
            
            ncsController = NcsController(this.controller, this.defaultInput);
            ncsController.initStatisticsRecording(30, this.dimX);
            
            % assume a measurement that is still applicable
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay;
            measDelay = timestep - measTime;
            this.assertEqual(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
            previousMode = [];
            
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
            numUsedMeas = ncsController.statistics.numUsedMeasurements(timestep);
            numDiscardedMeas = ncsController.statistics.numDiscardedMeasurements(timestep);
            actualControllerState = ncsController.statistics.controllerStates(:, timestep + 1);
            
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
        
        %% testChangeCaDelayProbsFalse
        function testChangeCaDelayProbsFalse(this)
            % this is currently generally not possible for most controllers
            % in use
            
            ncsController = NcsController(this.controller, this.defaultInput);
            
            newDelayProbs = [this.delayProbs(:); 0];
            this.verifyFalse(ncsController.changeCaDelayProbs(newDelayProbs));
            
            % false should also be returned if invalid distribution is passed
            newDelayProbs = eye(4);
            this.verifyFalse(ncsController.changeCaDelayProbs(newDelayProbs));            
        end
        
        %% testChangeCaDelayProbsTrue
        function testChangeCaDelayProbsTrue(this)
            % for the IMM-based controller, this operation is supported
            horizonlength = 5;
            immBasedController = IMMBasedRecedingHorizonController(this.A, this.B, this.C, this.Q, this.R, this.delayProbs, ...
                this.controlSeqLength, this.maxMeasDelay, this.W, Gaussian(zeros(this.dimY, 1), this.V), ...
                horizonlength, zeros(this.dimX, 1), eye(this.dimX));
            ncsController = NcsController(immBasedController, this.defaultInput);
            newDelayProbs = [this.delayProbs(:); 0];
            this.verifyTrue(ncsController.changeCaDelayProbs(newDelayProbs));
        end
    end
end

