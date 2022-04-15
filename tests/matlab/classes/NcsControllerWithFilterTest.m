classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsControllerWithFilterTest < matlab.unittest.TestCase
    % Test cases for NcsControllerWithFilter.
    
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
        
        controller;
        filter;
        filterPlantModel;
        filterSensorModel;
        delayWeights;
        defaultInput;
        initialCaDelayDistribution;
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
            this.delayWeights = ones(this.controlSeqLength + 1, 1) / (this.controlSeqLength + 1);
            this.defaultInput = 2 + zeros(this.dimU, 1);
            this.initialCaDelayDistribution = ones(this.controlSeqLength + 2, 1) / (this.controlSeqLength + 2);
            
            this.filterSensorModel = LinearMeasurementModel(this.C);
            this.filterSensorModel.setNoise(Gaussian(zeros(this.dimY, 1), this.V));
            this.filterPlantModel = LinearPlant(this.A, this.B, this.W);
            
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.filter = DelayedKF(this.maxMeasDelay, eye(this.controlSeqLength + 1));
        end
    end
    
    methods (Test)
        %% testNcsControllerWithFilter
        function testNcsControllerWithFilter(this)
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEmpty(ncsController.plantStateOrigin);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyFalse(ncsController.isEventBased);
            this.verifySameHandle(ncsController.filter, this.filter);
            this.verifySameHandle(ncsController.plantModel, this.filterPlantModel);
            this.verifySameHandle(ncsController.measModel, this.filterSensorModel);
                        
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyFalse(ncsController.isEventBased);
            this.verifySameHandle(ncsController.filter, this.filter);
            this.verifySameHandle(ncsController.plantModel, this.filterPlantModel);
            this.verifySameHandle(ncsController.measModel, this.filterSensorModel);
            
            this.verifyTrue(ncsController.isControllerStateAdmissible());                        
        end
        
        %% testStepNoMode
        function testStepNoMode(this)
            expectedErrId = 'NcsControllerWithFilter:Step:MissingPreviousPlantMode';
            
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            
            modePlantModels = arrayfun(@(~) LinearPlant(this.A, this.B, this.W), 1:this.controlSeqLength + 1, ...
                'UniformOutput', false);                                   
            jumpLinearPlantModel = JumpLinearSystemModel(this.controlSeqLength + 1, modePlantModels);
            
            % use the dummy filter here
            ineptFilter = DelayedMeasurementsFilterStub(this.maxMeasDelay, 'Dummy Filter');
            ineptFilter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, ineptFilter, ...
                jumpLinearPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
                        
            acPackets = [];
            scPackets = [];
            timestep = 1; % initial timestep
            previousMode = [];
            
            this.verifyError(@() ncsController.step(timestep, scPackets, acPackets, previousMode), expectedErrId);
        end
        
        %% testStepNoPacketsZeroState
        function testStepNoPacketsZeroState(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            previousMode = 1;
            
            % 1 time step = 1 second
            % perform 20 steps, without measurement and acks, so that
            % statistics are recorded properly
            for j=1:20
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end

            % reset the state to zero            
            this.filter.setState(controllerState)
                        
            timestep = 21;                         
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
                        
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: controller is linear, prediction is noise-free and
            % state is zero, so inputs should be zero
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength), 'AbsTol', 1e-5);
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            this.verifyEqual(actualControllerState, plantState, 'AbsTol', 1e-5);
            this.verifyEqual(numUsedMeas, 0);
            this.verifyEqual(numDiscardedMeas, 0);
        end
        
        %% testStepZeroStateMeasDiscarded
        function testStepZeroStateMeasDiscarded(this)                          
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            previousMode = 1;
            
            % 1 time step = 1 second
            % perform 20 steps, without measurement and acks, so that
            % statistics are recorded properly
            for j=1:20
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end

            % reset the state to zero            
            this.filter.setState(controllerState)
            
            timestep = 21;
            % assume a measurement that is too old
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay -1;
            measDelay = timestep - measTime;
            this.assertGreaterThan(measDelay, this.maxMeasDelay);
            
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
                        
            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
            
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: inputs should not be zero
            % as state is predicted
            this.verifyNotEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            % the state should have changed now, as we predict
            this.verifyNotEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 0);
            this.verifyEqual(numDiscardedMeas, 1);
        end
        
        %% testStepZeroStateMeasUsed
        function testStepZeroStateMeasUsed(this)
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            ncsController.initStatisticsRecording();
            
            acPackets = [];
            scPackets = [];
            previousMode = 1;
            
            % 1 time step = 1 second
            % perform 20 steps, without measurement and acks, so that
            % statistics are recorded properly
            for j=1:20
                ncsController.step(j, scPackets, acPackets, previousMode, j);
            end

            % reset the state to zero            
            this.filter.setState(controllerState)
            
            timestep = 21;
            % assume a measurement that is still applicable
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay;
            measDelay = timestep - measTime;
            this.assertEqual(measDelay, this.maxMeasDelay);
                        
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;

            dataPacket = ncsController.step(timestep, scPackets, acPackets, previousMode, timestep);
            
            [numUsedMeas, numDiscardedMeas, actualControllerState] = ncsController.getStatisticsForTimestep(timestep);
                        
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: inputs should not be zero
            % as state is updated
            this.verifyNotEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            % the state should have changed now, as we updated it
            this.verifyNotEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 1);
            this.verifyEqual(numDiscardedMeas, 0);
        end
        
        %% testChangeSequenceLength
        function testChangeSequenceLength(this)
            newSeqLength = this.controlSeqLength + 1;
            
            % with the filter in use this is not possible, although the
            % controller supports this
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            this.assertEqual(ncsController.controller.sequenceLength, this.controlSeqLength);           
            
            this.verifyFalse(ncsController.changeSequenceLength(newSeqLength));
            this.verifyEqual(this.controller.sequenceLength, this.controlSeqLength);           
        end
        
        %% testChangeCaDelayProbsInvalidProbs
        function testChangeCaDelayProbsInvalidProbs(this)
            expectedErrId = 'Validator:ValidateDiscreteProbabilityDistribution:InvalidProbs';
            
            % the controller does not use the delay probs, but the filter
            % (DelayedKF) does, so it should be possible to change the
            % distribution of the delays
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
           
            % call should error in case of invalid distribution
            invalidDistribution = eye(4);
            this.verifyError(@() ncsController.changeCaDelayProbs(invalidDistribution), expectedErrId);
        end
        
        %% testChangeCaDelayProbsTrue
        function testChangeCaDelayProbsTrue(this)
            mixinClass = ?CaDelayProbsChangeable;           
            this.assertFalse(ismember(mixinClass.Name, superclasses(this.controller)));
            
            % the controller does not use the delay probs, but the filter
            % (DelayedKF) does, so it should be possible to change the
            % distribution of the delays
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            
            newDelayProbs = [this.delayWeights(1:end-1); this.delayWeights(end) / 2; this.delayWeights(end) / 2];            
            this.verifyTrue(ncsController.changeCaDelayProbs(newDelayProbs));
        end
        
        %% testChangeCaDelayProbsFalse
        function testChangeCaDelayProbsFalse(this)
            % now use a controller that does not support this
            % initially, provide the controller with a mode transition
            % matrix
            modeTransitionMatrix = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(this.initialCaDelayDistribution, this.controlSeqLength + 1));
            
            newController = FiniteHorizonController(this.A, this.B, this.Q, this.R, ...
                modeTransitionMatrix, this.controlSeqLength, 1);
            ncsController = NcsControllerWithFilter(newController, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            
            mixinClass = ?CaDelayProbsChangeable;           
            this.assertFalse(ismember(mixinClass.Name, superclasses(newController)));
            
            
            newDelayProbs = [this.delayWeights(1:end-1); this.delayWeights(end) / 2; this.delayWeights(end) / 2]; 
            this.verifyFalse(ncsController.changeCaDelayProbs(newDelayProbs));            
        end
%%
%%
         %% testChangeModelParametersFalse
        function testChangeModelParametersFalse(this)
            modeTransitionMatrix = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(this.initialCaDelayDistribution, this.controlSeqLength + 1));
            newController = FiniteHorizonController(this.A, this.B, this.Q, this.R, ...
                modeTransitionMatrix, this.controlSeqLength, 1);
            
            mixinClass = ?ModelParamsChangeable;
            this.assertFalse(ismember(mixinClass.Name, superclasses(newController)));
            this.assertEqual(full(this.filterPlantModel.sysMatrix(1:this.dimX, 1:this.dimX)), this.A);
            
            newA = this.A * 2;
            newB = this.B * 2;
            newW = this.W * 2;
            
            ncsController = NcsControllerWithFilter(newController, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            
            this.verifyFalse(ncsController.changeModelParameters(newA, newB, newW));
            % no effect at all
            this.verifyEqual(full(this.filterPlantModel.sysMatrix(1:this.dimX, 1:this.dimX)), this.A);
        end
        
        %% testChangeModelParametersTrue
        function testChangeModelParametersTrue(this)
            mixinClass = ?ModelParamsChangeable;           
            
            this.assertTrue(ismember(mixinClass.Name, superclasses(this.controller)));
            this.assertEqual(full(this.filterPlantModel.sysMatrix(1:this.dimX, 1:this.dimX)), this.A);
            
            newA = this.A * 2;
            newB = this.B * 2;
            newW = this.W * 2;
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.initialCaDelayDistribution);
            
            this.verifyTrue(ncsController.changeModelParameters(newA, newB, newW));
            
            noise = this.filterPlantModel.noise;
            [noiseMean, noiseCov] = noise.getMeanAndCov();
            % check the side effect
            this.verifyEqual(full(this.filterPlantModel.sysMatrix(1:this.dimX, 1:this.dimX)), newA);
            this.verifyClass(noise, ?Gaussian);
            this.verifyEqual(noiseMean, zeros(this.dimX, 1));
            this.verifyEqual(noiseCov, newW);
        end
    end
end

