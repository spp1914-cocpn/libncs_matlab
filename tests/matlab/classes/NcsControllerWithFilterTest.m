classdef NcsControllerWithFilterTest < matlab.unittest.TestCase
    % Test cases for NcsControllerWithFilter.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018  Florian Rosenthal <florian.rosenthal@kit.edu>
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
            
            this.filterSensorModel = LinearMeasurementModel(this.C);
            this.filterSensorModel.setNoise(Gaussian(zeros(this.dimY, 1), this.V));
            this.filterPlantModel = DelayedKFSystemModel(this.A, this.B, Gaussian(zeros(this.dimX, 1), this.W), ...
                this.controlSeqLength + 1, this.maxMeasDelay, this.delayWeights);
            
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.filter = DelayedKF(this.maxMeasDelay);
        end
    end
    
    methods (Test)
        %% testNcsControllerWithFilter
        function testNcsControllerWithFilter(this)
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEmpty(ncsController.plantStateOrigin);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyFalse(ncsController.isEventBased);
            this.verifySameHandle(ncsController.filter, this.filter);
            this.verifySameHandle(ncsController.plantModel, this.filterPlantModel);
            this.verifySameHandle(ncsController.measModel, this.filterSensorModel);
                        
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput, this.plantStateOrigin);
            
            this.verifySameHandle(ncsController.controller, this.controller);
            this.verifyEqual(ncsController.plantStateOrigin, this.plantStateOrigin);
            this.verifyEqual(ncsController.controlSequenceLength, this.controlSeqLength);
            this.verifyFalse(ncsController.isEventBased);
            this.verifySameHandle(ncsController.filter, this.filter);
            this.verifySameHandle(ncsController.plantModel, this.filterPlantModel);
            this.verifySameHandle(ncsController.measModel, this.filterSensorModel);
                        
        end
        
        %% testStepNoMode
        function testStepNoMode(this)
            expectedErrId = 'NcsControllerWithFilter:Step:MissingPreviousPlantMode';
            
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
                        
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
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
                        
            acPackets = [];
            scPackets = [];
            timestep = 1; % initial timestep
            previousMode = 1;
            
            [dataPacket, numUsedMeas, numDiscardedMeas, actualControllerState] ...
                = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
            this.verifyNotEmpty(dataPacket);
            this.verifyClass(dataPacket, ?DataPacket);
            
            % check payload, timestep, source (id=2) and destination (id=1) of
            % packet
            inputSequence = dataPacket.payload;
            this.verifySize(inputSequence, [this.dimU this.controlSeqLength]);
            % sanity check for control sequence: controller is linear, prediction is noise-free and
            % state is zero, so inputs should be zero
            this.verifyEqual(inputSequence, zeros(this.dimU, this.controlSeqLength));
            this.verifyEqual(dataPacket.sourceAddress, 2);
            this.verifyEqual(dataPacket.destinationAddress, 1);
            this.verifyEqual(dataPacket.timeStamp, timestep);
            
            this.verifyEqual(actualControllerState, plantState);
            this.verifyEqual(numUsedMeas, 0);
            this.verifyEqual(numDiscardedMeas, 0);
        end
        
        %% testStepZeroStateMeasDiscarded
        function testStepZeroStateMeasDiscarded(this)              
            timestep = 21;
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
            
            % assume a measurement that is too old
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay -1;
            measDelay = timestep - measTime;
            this.assertGreaterThan(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;
            previousMode = 1;
            
            [dataPacket, numUsedMeas, numDiscardedMeas, actualControllerState] ...
                = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
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
            timestep = 21;
            previousMode = 1;
            plantState = zeros(this.dimX, 1);
            controllerState = Gaussian(plantState, eye(this.dimX));
            this.filter.setState(controllerState);
            
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
            
            % assume a measurement that is still applicable
            measurement = 2 + zeros(this.dimY, 1);
            measTime = timestep - this.maxMeasDelay;
            measDelay = timestep - measTime;
            this.assertEqual(measDelay, this.maxMeasDelay);
            
            acPackets = [];
            scPackets = DataPacket(measurement, measTime);
            scPackets.packetDelay = measDelay;

            [dataPacket, numUsedMeas, numDiscardedMeas, actualControllerState] ...
                = ncsController.step(timestep, scPackets, acPackets, previousMode);
            
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
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
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
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
           
            % call should error in case of invalid distribution
            invalidDistribution = eye(4);
            this.verifyError(@() ncsController.changeCaDelayProbs(invalidDistribution), expectedErrId);
        end
        
        %% testChangeCaDelayDistribution
        function testChangeCaDelayDistribution(this)
            % the controller does not use the delay probs, but the filter
            % (DelayedKF) does, so it should be possible to change the
            % distribution of the delays
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
            
            newDelayProbs = [this.delayWeights(1:end-1); this.delayWeights(end) / 2; this.delayWeights(end) / 2];            
            this.verifyTrue(ncsController.changeCaDelayProbs(newDelayProbs));
            
            % now use a controller that does not support this
            newController = FiniteHorizonController(this.A, this.B, this.Q, this.R, ...
                this.delayWeights, this.controlSeqLength, 1);
            ncsController = NcsControllerWithFilter(newController, this.filter, ...
                this.filterPlantModel, this.filterSensorModel, this.defaultInput);
            
            this.verifyFalse(ncsController.changeCaDelayProbs(newDelayProbs));
            
        end
    end
end
