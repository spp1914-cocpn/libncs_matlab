classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsInitializeTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_initialize.
    
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
        matFilename;
        translatorFilename;
        configStruct;
        id;
        
        maxSimTime;
        dimX;
        dimU;
        dimY;
        
        samplingInterval;
        maxMeasDelay;
        maxControlSequenceDelay;
        controlSeqLength;
        initialPlantState;
        caDelayProbs;
        controllerClassName;
        A;
        B;
        C;
        W;
        V;
        Q
        R;
        
        dummyFile;
        
        ncsTranslator;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(~)
            ComponentMap.getInstance().clear();
            % required to destroy the singleton instance
            clear ComponentMap;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)            
            this.maxSimTime = 1e12;
            
            this.dimX = 3;
            this.dimU = 2;
            this.dimY = 1;           
            
            this.id = 'Test NCS';
            this.samplingInterval = 0.1;
            this.A = 0.75 * eye(this.dimX);
            this.B = ones(this.dimX, this.dimU);
            this.C = [1 2 3];
            this.W = eye(this.dimX); % sys noise cov
            this.V = 0.1^2; % variance of the meas noise
            this.Q = 2 * eye(this.dimX);
            this.R = 0.5 * eye(this.dimU);
            this.controlSeqLength = 5;
            this.maxMeasDelay = 10;
            this.maxControlSequenceDelay = 10;
            this.caDelayProbs = [0.5 0.4 0.1];
            this.initialPlantState = zeros(this.dimX, 1); % plant state is already at the origin
            this.controllerClassName = 'NominalPredictiveController';
            
            qocRateCurve = cfit(fittype('a*x^2+b*x+c'), 1, 2, 3);
            controlErrorQocCurve = cfit(fittype('-a*x+2'), 0.5); % for simplicity, use linear function
            maxDataRate = 5.5;
            
            this.ncsTranslator = NcsTranslator(qocRateCurve, controlErrorQocCurve, maxDataRate);
            
            import matlab.unittest.fixtures.WorkingFolderFixture;
            
            this.applyFixture(WorkingFolderFixture);
            this.matFilename = [pwd filesep 'test.mat'];
            this.translatorFilename = [pwd filesep 'translator.mat'];
            
            % add some content into the translator file, but not an
            % NcsTranslator
            translatorFile = matfile(this.translatorFilename, 'Writable', true);
            translatorFile.dummy = 42;
            
            this.dummyFile = [pwd filesep 'dummy.txt'];
            fid = fopen(this.dummyFile, 'w');
            fprintf(fid, 'dummy');
            fclose(fid);
            
            configFile = matfile(this.matFilename, 'Writable', true);
            % A, B, in config describe plant model as used by controller
            configFile.A = this.A; 
            configFile.B = this.B;
            configFile.C = this.C;
            configFile.W = this.W;
            configFile.V = this.V;
            configFile.Q = this.Q;
            configFile.R = this.R;
            configFile.samplingInterval = this.samplingInterval;
            configFile.plantSamplingInterval = this.samplingInterval;
            configFile.controlSequenceLength = this.controlSeqLength;
            configFile.maxMeasDelay = this.maxMeasDelay;
            configFile.maxControlSequenceDelay = this.maxControlSequenceDelay;
            
            configFile.plant = LinearPlant(this.A, this.B, this.W);
            
            this.configStruct.controllerClassName = this.controllerClassName;
            this.configStruct.initialPlantState = this.initialPlantState;
            this.configStruct.caDelayProbs = this.caDelayProbs;
            
            this.addTeardown(@() this.tearDown());
        end
    end
    
    methods (Test)
        %% testInvalidConfigStruct
        function testInvalidConfigStruct(this)
            expectedErrId = 'ncs_initialize:InvalidConfigStruct';
            
            invalidConfigStruct = 42; % not a struct;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, invalidConfigStruct, this.matFilename), ...
                expectedErrId);
            
            invalidConfigStruct = [this.configStruct; this.configStruct]; % struct array
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, invalidConfigStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testInvalidFilename
        function testInvalidFilename(this)
            expectedErrId = 'ncs_initialize:InvalidFilename';
            
            invalidFilename = 42; % not a char vector
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, invalidFilename), ...
                expectedErrId);
            
            invalidFilename = []; % empty matrix
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, invalidFilename), ...
                expectedErrId);
        end
    
        %% testInvalidFile
        function testInvalidFile(this)
            % ensure that file we use later exists
            this.assertTrue(exist(this.dummyFile, 'file') == 2);
            
            expectedErrId = 'ncs_initialize:InvalidFile';
            
            invalidFile = [pwd filesep 'test2.mat']; % does not exist
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, invalidFile), ...
                expectedErrId);
                        
            invalidFile = pwd; % not a file
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, invalidFile), ...
                expectedErrId);
            
            invalidFile = this.dummyFile; % exists, but incorrect extension
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, invalidFile), ...
                expectedErrId);
        end
        
        %% testInvalidConfig
        function testInvalidConfig(this)
            expectedErrId = 'ncs_initialize:CheckConfigFile';
            pattern = '** The following 3 variables'; % three variables supposed to be missing
 
            % manually investigate and check the exception
            try
                ncs_initialize(this.maxSimTime, this.id, [], this.matFilename);
            catch ex
                this.verifyEqual(ex.identifier, expectedErrId);
                errorMsg = ex.getReport('basic');                
                this.verifyNotEmpty(strfind(errorMsg, pattern));
            end            
        end
        
        %% testUnsupportedControllerClass
        function testUnsupportedControllerClass(this)
            expectedErrId = 'ncs_initialize:InitController:UnsupportedControllerClass';
            
            this.configStruct.controllerClassName = 'UnsupportedController'; % class is unknown
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testInitialEstimateMissing
        function testInitialEstimateMissing(this)
            expectedErrId = 'ncs_initialize:InitFilter:InitialEstimateMissing';
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            % we add a filter
            % but do not provide an initial estimate in the config
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testInvalidCaDelayProbs
        function testInvalidCaDelayProbs(this)
            expectedErrId = 'Validator:ValidateDiscreteProbabilityDistribution:InvalidProbs';
                        
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            invalidProbs = this.caDelayProbs(1:end-1); % does not sum to 1
            this.configStruct.caDelayProbs = invalidProbs;
            
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testUnsupportedFilterClass
        function testUnsupportedFilterClass(this)
            expectedErrId = 'ncs_initialize:InitFilter:UnsupportedFilterClass';
            
            this.configStruct.filterClassName = 'NominalPredictiveController'; % not a filter
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testInvalidFiniteHorizonController
        function testInvalidFiniteHorizonController(this)
            expectedErrId = 'ncs_initialize:InitController:FiniteHorizonController';
            
            % we add constraint weightings, but no bounds
            this.configStruct.stateConstraintWeightings = ones(this.dimX, this.dimX, 2);
            this.configStruct.inputConstraintWeightings = ones(this.dimU, this.dimU, 2);
            this.configStruct.controllerClassName = 'FiniteHorizonController';
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end
        
        %% testInvalidFiniteHorizonTrackingController
        function testInvalidFiniteHorizonTrackingController(this)
            expectedErrId = 'ncs_initialize:InitController:FiniteHorizonTrackingController';
            
            % we add a Z matrix, but no ref trajectory
            this.configStruct.Z = repmat(2, 1, this.dimX);
            this.configStruct.controllerClassName = 'FiniteHorizonTrackingController';
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
            
            % we add a  ref trajectory, but no Z matrix
            this.configStruct = rmfield(this.configStruct, 'Z');
            this.configStruct.refTrajectory = ones(1, 100);
            this.configStruct.controllerClassName = 'FiniteHorizonTrackingController';
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end      
        
        %% testInvalidEventTriggeredInfiniteHorizonController
        function testInvalidEventTriggeredInfiniteHorizonController(this)
            expectedErrId = 'ncs_initialize:InitController:EventTriggeredInfiniteHorizonController';
            
             % we do not pass transmision costs
            this.configStruct.controllerClassName = 'EventTriggeredInfiniteHorizonController';
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);              
        end
        
        %% testInvalidInfiniteHorizonUdpLikeController
        function testInvalidInfiniteHorizonUdpLikeController(this)
            expectedErrId = 'ncs_initialize:InitController:InfiniteHorizonUdpLikeController';
            
            % we do not pass delays for the sc link
            this.configStruct.controllerClassName = 'InfiniteHorizonUdpLikeController';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);    
            
            % we do not pass an initial estimate
            this.configStruct.controllerClassName = 'InfiniteHorizonUdpLikeController';
            this.configStruct = rmfield(this.configStruct, 'initialEstimate');
            this.configStruct.scDelayProbs = this.caDelayProbs;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId); 
        end
        
        %% testInvalidIMMBasedRecedingHorizonController
        function testInvalidIMMBasedRecedingHorizonController(this)
            expectedErrId = 'ncs_initialize:InitController:IMMBasedRecedingHorizonController';
         
            % we do not pass an initial estimate
            this.configStruct.controllerClassName = 'IMMBasedRecedingHorizonController';            
            this.configStruct.scDelayProbs = this.caDelayProbs;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId); 
        end
        
        %% testInvalidRecedingHorizonUdpLikeController
        function testInvalidRecedingHorizonUdpLikeController(this)
            expectedErrId = 'ncs_initialize:InitController:RecedingHorizonUdpLikeController';
            
            % we do not pass delays for the sc link
            this.configStruct.controllerClassName = 'RecedingHorizonUdpLikeController';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);    
            
            % we do not pass an initial estimate
            this.configStruct.controllerClassName = 'RecedingHorizonUdpLikeController';
            this.configStruct = rmfield(this.configStruct, 'initialEstimate');
            this.assertFalse(isfield(this.configStruct, 'initialEstimate'));
            
            this.configStruct.scDelayProbs = this.caDelayProbs;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId); 
        end
        
        %% testNoFilterClassName
        function testNoFilterClassName(this)
            expectedErrId = 'ncs_initialize:FilterClassNameMissing';
            % the chosen controller needs a filter
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);            
        end
        
        %% testInvalidTranslatorFile
        function testInvalidTranslatorFile(this)
            nonExistingFile = [pwd filesep 'test2.mat'];
            this.assertTrue(exist(nonExistingFile, 'file') == 0);
            % ensure that file we use later exists
            this.assertTrue(exist(this.dummyFile, 'file') == 2);  
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            expectedErrId = 'ncs_initialize:InvalidTranslatorFile';                     
                        
            invalidTranslatorFile = nonExistingFile; % does not exist
            this.configStruct.translatorFile = invalidTranslatorFile;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
                        
            invalidTranslatorFile = pwd; % not a file
            this.configStruct.translatorFile = invalidTranslatorFile;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
            
            invalidTranslatorFile = this.dummyFile; % exists, but incorrect extension
            this.configStruct.translatorFile = invalidTranslatorFile;
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);            
        end
        
        %% testInvalidCheckTranslatorFile
        function testInvalidCheckTranslatorFile(this)
            % mat file is existing
            this.assertTrue(exist(this.translatorFilename, 'file') == 2);
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            expectedErrId = 'ncs_initialize:CheckTranslatorFile';
            
            this.configStruct.translatorFile = this.translatorFilename; % wrong content
            this.verifyError(@() ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
        end        
%%
%%
        %% test
        function test(this)
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            this.configStruct.plantSamplingInterval = this.samplingInterval / 10; % make the plant 10 times faster
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            this.verifyGreaterThan(ncsHandle, 0);
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, this.samplingInterval / 10);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEmpty(ncs.controller.plantStateOrigin);
            
            this.verifyClass(ncs.sensor, ?NcsSensor);
            
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyEqual(ncs.plant.dimState, this.dimX);
            this.verifyEqual(ncs.plant.dimInput, this.dimU);
            
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
        end        
        
        %% testEventBasedSensor
        function testEventBasedSensor(this)
            expectedMeasDelta = 42;
            
            this.configStruct.sensorEventBased = true;
            this.configStruct.sensorMeasDelta = expectedMeasDelta;
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEmpty(ncs.controller.plantStateOrigin);
            
            this.verifyClass(ncs.sensor, ?EventBasedNcsSensor);
            this.verifyEqual(ncs.sensor.measurementDelta, expectedMeasDelta);
            
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyEqual(ncs.plant.dimState, this.dimX);
            this.verifyEqual(ncs.plant.dimInput, this.dimU);
            
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
        end
        
        %% testEventBasedSensorDefaultMeasDelta
        function testEventBasedSensorDefaultMeasDelta(this)
            expectedMeasDelta = 10; % the default value defined in EventBasedNcsSensor
            
            this.configStruct.sensorEventBased = true;
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
                 
            this.verifyClass(ncs.sensor, ?EventBasedNcsSensor);
            this.verifyEqual(ncs.sensor.measurementDelta, expectedMeasDelta);
        end
        
        %% testControllerWithFilter
        function testControllerWithFilter(this)
            expectedMean = zeros(this.dimX, 1);
            expectedCov = eye(this.dimX);
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, expectedCov);
            this.configStruct.controllerEventBased = false; 
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEmpty(ncs.controller.plantStateOrigin);  
            
            this.verifyClass(ncs.controller.filter, ?DelayedModeIMMF);
            [actualMean, actualCov] = ncs.controller.filter.getStateMeanAndCov();
            this.verifyEqual(actualMean, expectedMean, 'AbsTol', 1e-8);
            this.verifyEqual(actualCov, expectedCov, 'AbsTol', 1e-8);            
        end
        
        %% testControllerWithFilterSystemNoiseMatrix
        function testControllerWithFilterSystemNoiseMatrix(this)
            expectedMean = zeros(this.dimX, 1);
            expectedSysNoiseMatrix = [1 0 0; 0 0 0; 0 0 0]; % G matrix
            plantState = [1 1 1]';
            timestep = 1;
            simTimeSec = timestep; % for simplicty 1 time step = 1s
                        
            % we use a different plant now
            this.configStruct.plant = LinearPlant(this.A, this.B, this.W, expectedSysNoiseMatrix);
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, eye(this.dimX));
            this.configStruct.controllerEventBased = false; 
            this.configStruct.G = expectedSysNoiseMatrix;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);

            % no inputs applied
            ncs.plant.init(plantState, timestep);
            newPlantState = ncs.plant.plantStep(timestep,simTimeSec);
          
            % check if the state evolved correctly; noise affects only
            % first component
            predState = this.A * plantState;
            this.verifyEqual(newPlantState(2:end), predState(2:end));  
        end
        
        %% testControllerWithFilterLinearizationPoint
        function testControllerWithFilterLinearizationPoint(this)
            expectedMean = zeros(this.dimX, 1);
            expectedCov = eye(this.dimX);
            expectedPlantStateOrigin = 2 * ones(this.dimX, 1);
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, expectedCov);
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEqual(ncs.controller.plantStateOrigin, expectedPlantStateOrigin);
            
            this.verifyClass(ncs.controller.filter, ?DelayedModeIMMF);
            [actualMean, actualCov] = ncs.controller.filter.getStateMeanAndCov();
            this.verifyEqual(actualMean, expectedMean, 'AbsTol', 1e-8);
            this.verifyEqual(actualCov, expectedCov, 'AbsTol', 1e-8);            
        end
        
        %% testControllerWithFilterEventBased
        function testControllerWithFilterEventBased(this)
            expectedMean = ones(this.dimX, 1);
            expectedCov = 0.5 * eye(this.dimX);
            expectedControllerDeadband = 50;
            expectedControllerEventTrigger = EventBasedControllerTriggerCriterion.Sequence;
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, expectedCov);
            this.configStruct.controllerEventBased = true;
            this.configStruct.controllerDeadband = expectedControllerDeadband;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?EventBasedNcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEmpty(ncs.controller.plantStateOrigin);
            this.verifyEqual(ncs.controller.deadband, expectedControllerDeadband);
            this.verifyEqual(ncs.controller.eventTrigger, expectedControllerEventTrigger);
            
            this.verifyClass(ncs.controller.filter, ?DelayedModeIMMF);
            [actualMean, actualCov] = ncs.controller.filter.getStateMeanAndCov();
            this.verifyEqual(actualMean, expectedMean, 'AbsTol', 1e-8);
            this.verifyEqual(actualCov, expectedCov, 'AbsTol', 1e-8);            
        end
        
        %% testControllerWithFilterEventBasedLinearizationPoint
        function testControllerWithFilterEventBasedLinearizationPoint(this)
            expectedMean = ones(this.dimX, 1);
            expectedCov = 0.5 * eye(this.dimX);
            expectedControllerDeadband = 50;
            expectedControllerEventTrigger = EventBasedControllerTriggerCriterion.ControlError;
            expectedPlantStateOrigin = 2 * ones(this.dimX, 1);
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, expectedCov);
            this.configStruct.controllerEventBased = true;
            this.configStruct.controllerDeadband = expectedControllerDeadband;
            this.configStruct.controllerEventTrigger = expectedControllerEventTrigger;
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?EventBasedNcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.Q, this.Q);
            this.verifyEqual(ncs.controller.controller.R, this.R);
            this.verifyEmpty(ncs.controller.controller.setpoint);
            this.verifyEqual(ncs.controller.plantStateOrigin, expectedPlantStateOrigin);
            this.verifyEqual(ncs.controller.deadband, expectedControllerDeadband);
            this.verifyEqual(ncs.controller.eventTrigger, expectedControllerEventTrigger);
                        
            this.verifyClass(ncs.controller.filter, ?DelayedModeIMMF);
            [actualMean, actualCov] = ncs.controller.filter.getStateMeanAndCov();
            this.verifyEqual(actualMean, expectedMean, 'AbsTol', 1e-8);
            this.verifyEqual(actualCov, expectedCov, 'AbsTol', 1e-8);            
        end
        
        %% testControllerLinearizationPoint
        function testControllerLinearizationPoint(this)
            import matlab.unittest.fixtures.SuppressedWarningsFixture
            
            this.applyFixture(...
                SuppressedWarningsFixture('ncs_initialize:InitController:IMMBasedRecedingHorizonController:NoHorizonLength'));
            
            expectedPlantStateOrigin = ones(this.dimX, 1);
                       
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.controllerClassName = 'IMMBasedRecedingHorizonController';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
                        
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?NcsController);
            this.verifyClass(ncs.controller.controller, ?IMMBasedRecedingHorizonController);            
            this.verifyNotEmpty(ncs.controller.plantStateOrigin);
            this.verifyEqual(ncs.controller.plantStateOrigin, expectedPlantStateOrigin);
                                 
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
        end
        
        %% testControllerEventBasedLinearizationPoint
        function testControllerEventBasedLinearizationPoint(this)
            import matlab.unittest.fixtures.SuppressedWarningsFixture
            
            this.applyFixture(...
                SuppressedWarningsFixture('ncs_initialize:InitController:IMMBasedRecedingHorizonController:NoHorizonLength'));
            
            this.configStruct.controllerClassName = 'IMMBasedRecedingHorizonController';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            expectedPlantStateOrigin = ones(this.dimX, 1);
            expectedControllerDeadband = 50;
            expectedControllerEventTrigger = EventBasedControllerTriggerCriterion.ControlError;
            
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.controllerEventBased = true;
            this.configStruct.controllerDeadband = expectedControllerDeadband;
            this.configStruct.controllerEventTrigger = expectedControllerEventTrigger;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);
            this.verifyEqual(ncs.samplingInterval, this.samplingInterval);
            this.verifyEqual(ncs.name, this.id);
            
            this.verifyClass(ncs.controller, ?EventBasedNcsController);
            this.verifyClass(ncs.controller.controller, ?IMMBasedRecedingHorizonController);
            this.verifyNotEmpty(ncs.controller.plantStateOrigin);
            this.verifyEqual(ncs.controller.plantStateOrigin, expectedPlantStateOrigin);
            this.verifyEqual(ncs.controller.deadband, expectedControllerDeadband);
            this.verifyEqual(ncs.controller.eventTrigger, expectedControllerEventTrigger);
                     
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
        end
        
        %% testPlantInvertedPendulum
        function testPlantInvertedPendulum(this)
            expectedSamplingInterval = this.samplingInterval / 2;
            expectedPlantSamplingInterval = this.samplingInterval / 2;
            expectedWcontLin = blkdiag(0.1, 0.5); % noise in the continuous-time linearization
            expectedPendNoiseCov = gallery('moler', 2); % noise cov (nonlinear pendulum dynamics)
            
            
            pendulum = InvertedPendulum(1, 1, 1, 0.1, this.samplingInterval);
            pendulum.varDisturbanceForcePendulumContLin = expectedWcontLin(1, 1);
            pendulum.varDisturbanceForceActuatorContLin = expectedWcontLin(2, 2);
            pendulumCopyNoNoise = InvertedPendulum(1, 1, 1, 0.1, this.samplingInterval);
            
            expectedPlantStateOrigin = [0 0 pi 0]'; % pendulum state is 4-dimensional
            
            [A_d, B_d, C_d, ~] = pendulum.linearizeAroundUpwardEquilibrium(); % discrete time matrices
            [expA, expB, expC, ~] = pendulum.linearizeAroundUpwardEquilibrium(expectedSamplingInterval);
                                           
            this.configStruct.A = A_d;
            this.configStruct.B = B_d;
            this.configStruct.C = C_d;            
            this.configStruct.V = eye(2); % two-dimensional measurements
            this.configStruct.samplingInterval = expectedSamplingInterval;
            this.configStruct.plantSamplingInterval = expectedPlantSamplingInterval;
            this.configStruct.Q = eye(4);
            this.configStruct.R = 100;
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.plant = pendulumCopyNoNoise;
            this.configStruct.W = expectedWcontLin; % inject the noise into the config here (for the linearization)
            this.configStruct.W_pend = expectedPendNoiseCov; % for the nonlinear dynamics
            this.configStruct.initialPlantState = zeros(4, 1); % initial pendulum state
            
            this.configStruct.controllerClassName = 'NominalPredictiveController';
            this.configStruct.filterClassName = 'DelayedKF';
            this.configStruct.initialEstimate = Gaussian(zeros(4, 1), eye(4));
            expectedGain = -dlqr(expA, expB, this.configStruct.Q, this.configStruct.R);
            
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);            
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);            
            this.verifyEqual(ncs.name, this.id);
            this.verifyEqual(ncs.samplingInterval, expectedSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, expectedPlantSamplingInterval);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.L, expectedGain, 'AbsTol', 1e-8);
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength)
            
            % check if the filter uses the correct model (linearized dynamics) incl. noise
            [A_cont, ~, ~, G_cont, ~] = pendulum.linearizeAroundUpwardEquilibriumCont();
            expectedWdisc = integral(@(x) expm(A_cont*x) * G_cont * expectedWcontLin * G_cont' * expm(A_cont'*x), ...
                    0, expectedPlantSamplingInterval, 'ArrayValued', true);  
            this.verifyClass(ncs.controller.plantModel, ?LinearPlant);
            this.verifyEqual(ncs.controller.plantModel.sysMatrix, expA);
            this.verifyEqual(ncs.controller.plantModel.inputMatrix, expB);
            this.verifyClass(ncs.controller.plantModel.noise, ?Gaussian);
            [filterNoiseMean, filterNoiseCov] = ncs.controller.plantModel.noise.getMeanAndCov();
            this.verifyEqual(filterNoiseMean, zeros(4,1));
            this.verifyEqual(filterNoiseCov, expectedWdisc, 'AbsTol', 1e-8);
            
            % check the plant (inverted pendulum)
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyClass(ncs.plant.plant, ?InvertedPendulum);
            this.verifyEqual(ncs.plant.plant.samplingInterval, expectedPlantSamplingInterval);
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulumContLin, expectedWcontLin(1,1)); % noise affecting pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForceActuatorContLin, expectedWcontLin(2,2)); % noise affecting cart in the continuous-time linearization
            % two dimensional noise
            this.verifyClass(ncs.plant.plant.noise, ?Gaussian);            
            [plantNoiseMean, plantNoiseCov] = ncs.plant.plant.noise.getMeanAndCov();
            this.verifyEqual(plantNoiseMean, zeros(2, 1));
            this.verifyEqual(plantNoiseCov, expectedPendNoiseCov);
            
            % finally, check the measurement model assumed by the controller/filter
            this.verifyClass(ncs.controller.measModel, ?LinearMeasurementModel);
            this.verifyEqual(ncs.controller.measModel.measMatrix, expC);
            this.verifyClass(ncs.controller.measModel.noise, ?Gaussian);
            [measNoiseMean, measNoiseCov] = ncs.controller.measModel.noise.getMeanAndCov();
            this.verifyEqual(measNoiseMean, zeros(2, 1));
            this.verifyEqual(measNoiseCov, this.configStruct.V);
        end
        
        %% testPlantInvertedPendulumNoProcessNoise
        function testPlantInvertedPendulumNoProcessNoise(this)
            expectedSamplingInterval = this.samplingInterval / 2;
            expectedPlantSamplingInterval = this.samplingInterval / 2;
            expectedWcontLin = blkdiag(0.1, 0.5); % noise in the continuous-time linearization
                       
            
            pendulum = InvertedPendulum(1, 1, 1, 0.1, this.samplingInterval);
            pendulum.varDisturbanceForcePendulumContLin = expectedWcontLin(1, 1);
            pendulum.varDisturbanceForceActuatorContLin = expectedWcontLin(2, 2);
            pendulumCopyNoNoise = InvertedPendulum(1, 1, 1, 0.1, this.samplingInterval);
            
            expectedPlantStateOrigin = [0 0 pi 0]'; % pendulum state is 4-dimensional
            
            [A_d, B_d, C_d, ~] = pendulum.linearizeAroundUpwardEquilibrium(); % discrete time matrices
            [expA, expB, expC, ~] = pendulum.linearizeAroundUpwardEquilibrium(expectedSamplingInterval);
                                           
            this.configStruct.A = A_d;
            this.configStruct.B = B_d;
            this.configStruct.C = C_d;            
            this.configStruct.V = eye(2); % two-dimensional measurements
            this.configStruct.samplingInterval = expectedSamplingInterval;
            this.configStruct.plantSamplingInterval = expectedPlantSamplingInterval;
            this.configStruct.Q = eye(4);
            this.configStruct.R = 100;
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.plant = pendulumCopyNoNoise;
            this.configStruct.W = expectedWcontLin; % inject the noise into the config here (for the linearization)
            this.configStruct.usePlantNoise = false; % plant to be simulated without noise
            this.configStruct.initialPlantState = zeros(4, 1); % initial pendulum state
            
            this.configStruct.controllerClassName = 'NominalPredictiveController';
            this.configStruct.filterClassName = 'DelayedKF';
            this.configStruct.initialEstimate = Gaussian(zeros(4, 1), eye(4));
            expectedGain = -dlqr(expA, expB, this.configStruct.Q, this.configStruct.R);            
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);            
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);            
            this.verifyEqual(ncs.name, this.id);
            this.verifyEqual(ncs.samplingInterval, expectedSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, expectedPlantSamplingInterval);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.L, expectedGain, 'AbsTol', 1e-8);
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength)
            
            % check if the filter uses the correct model (linearized dynamics) incl. noise
            [A_cont, ~, ~, G_cont, ~] = pendulum.linearizeAroundUpwardEquilibriumCont();
            expectedWdisc = integral(@(x) expm(A_cont*x) * G_cont * expectedWcontLin * G_cont' * expm(A_cont'*x), ...
                    0, expectedPlantSamplingInterval, 'ArrayValued', true);  
            this.verifyClass(ncs.controller.plantModel, ?LinearPlant);
            this.verifyEqual(ncs.controller.plantModel.sysMatrix, expA);
            this.verifyEqual(ncs.controller.plantModel.inputMatrix, expB);
            this.verifyClass(ncs.controller.plantModel.noise, ?Gaussian);
            [filterNoiseMean, filterNoiseCov] = ncs.controller.plantModel.noise.getMeanAndCov();
            this.verifyEqual(filterNoiseMean, zeros(4,1));
            this.verifyEqual(filterNoiseCov, expectedWdisc, 'AbsTol', 1e-8);
            
            % plant is simulated without noise
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyClass(ncs.plant.plant, ?InvertedPendulum);
            this.verifyEqual(ncs.plant.plant.samplingInterval, expectedPlantSamplingInterval);
            this.verifyEmpty(ncs.plant.plant.noise);
            % buth the linearization has noise
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulumContLin, expectedWcontLin(1,1)); % noise affecting pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForceActuatorContLin, expectedWcontLin(2,2)); % noise affecting cart in the continuous-time linearization            
                        
            % finally, check the measurement model assumed by the controller/filter
            this.verifyClass(ncs.controller.measModel, ?LinearMeasurementModel);
            this.verifyEqual(ncs.controller.measModel.measMatrix, expC);
            this.verifyClass(ncs.controller.measModel.noise, ?Gaussian);
            [measNoiseMean, measNoiseCov] = ncs.controller.measModel.noise.getMeanAndCov();
            this.verifyEqual(measNoiseMean, zeros(2, 1));
            this.verifyEqual(measNoiseCov, this.configStruct.V);
        end
%%
%%
        %% testPlantDoublePendulum
        function testPlantDoublePendulum(this)
            dimPend = 6;
            dimW = 3;
            dimV = 3;
            expectedSamplingInterval = this.samplingInterval / 2;
            expectedPlantSamplingInterval = this.samplingInterval / 2;
            expectedWcontLin = blkdiag(0.1, 0.5, 0.2); % noise in the continuous-time linearization
            expectedPendNoiseCov = gallery('moler', dimW); % noise cov (nonlinear pendulum dynamics)
                        
            pendulum = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.1, 0.1, this.samplingInterval);
            pendulum.varDisturbanceForcePendulum1ContLin = expectedWcontLin(1, 1);
            pendulum.varDisturbanceForcePendulum2ContLin = expectedWcontLin(2, 2);
            pendulum.varDisturbanceForceActuatorContLin = expectedWcontLin(3, 3);
            pendulumCopyNoNoise = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.1, 0.1, this.samplingInterval);
            
            expectedPlantStateOrigin = [0 0 0 0 0 0]'; % pendulum state is 6-dimensional
            
            [A_d, B_d, C_d, ~] = pendulum.linearizeAroundUpwardEquilibrium(); % discrete time matrices
            [expA, expB, expC, ~] = pendulum.linearizeAroundUpwardEquilibrium(expectedSamplingInterval);
                                           
            this.configStruct.A = A_d;
            this.configStruct.B = B_d;
            this.configStruct.C = C_d;            
            this.configStruct.V = eye(dimV); % three-dimensional measurements
            this.configStruct.samplingInterval = expectedSamplingInterval;
            this.configStruct.plantSamplingInterval = expectedPlantSamplingInterval;
            this.configStruct.Q = eye(dimPend);
            this.configStruct.R = 100;
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.plant = pendulumCopyNoNoise;
            this.configStruct.W = expectedWcontLin; % inject the noise into the config here (for the linearization)
            this.configStruct.W_plant = expectedPendNoiseCov; % for the nonlinear dynamics
            this.configStruct.initialPlantState = zeros(dimPend, 1); % initial pendulum state
            
            this.configStruct.controllerClassName = 'NominalPredictiveController';
            this.configStruct.filterClassName = 'DelayedKF';
            this.configStruct.initialEstimate = Gaussian(zeros(dimPend, 1), eye(dimPend));
            expectedGain = -dlqr(expA, expB, this.configStruct.Q, this.configStruct.R);
                        
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);            
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);            
            this.verifyEqual(ncs.name, this.id);
            this.verifyEqual(ncs.samplingInterval, expectedSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, expectedPlantSamplingInterval);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.L, expectedGain, 'AbsTol', 1e-8);
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength)
            
            % check if the filter uses the correct model (linearized dynamics) incl. noise
            [A_cont, ~, ~, G_cont, ~] = pendulum.linearizeAroundUpwardEquilibriumCont();
            expectedWdisc = integral(@(x) expm(A_cont*x) * G_cont * expectedWcontLin * G_cont' * expm(A_cont'*x), ...
                    0, expectedPlantSamplingInterval, 'ArrayValued', true);  
            this.verifyClass(ncs.controller.plantModel, ?LinearPlant);
            this.verifyEqual(ncs.controller.plantModel.sysMatrix, expA);
            this.verifyEqual(ncs.controller.plantModel.inputMatrix, expB);
            this.verifyClass(ncs.controller.plantModel.noise, ?Gaussian);
            [filterNoiseMean, filterNoiseCov] = ncs.controller.plantModel.noise.getMeanAndCov();
            this.verifyEqual(filterNoiseMean, zeros(dimPend,1));
            this.verifyEqual(filterNoiseCov, expectedWdisc, 'AbsTol', 1e-8);
            
            % check the plant (inverted pendulum)
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyClass(ncs.plant.plant, ?DoubleInvertedPendulum);
            this.verifyEqual(ncs.plant.plant.samplingInterval, expectedPlantSamplingInterval);
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulum1ContLin, expectedWcontLin(1,1)); % noise affecting lower pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulum2ContLin, expectedWcontLin(2,2)); % noise affecting upper pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForceActuatorContLin, expectedWcontLin(3,3)); % noise affecting cart in the continuous-time linearization
            % three dimensional noise
            this.verifyClass(ncs.plant.plant.noise, ?Gaussian);            
            [plantNoiseMean, plantNoiseCov] = ncs.plant.plant.noise.getMeanAndCov();
            this.verifyEqual(plantNoiseMean, zeros(dimW, 1));
            this.verifyEqual(plantNoiseCov, expectedPendNoiseCov);
            
            % finally, check the measurement model assumed by the controller/filter
            this.verifyClass(ncs.controller.measModel, ?LinearMeasurementModel);
            this.verifyEqual(ncs.controller.measModel.measMatrix, expC);
            this.verifyClass(ncs.controller.measModel.noise, ?Gaussian);
            [measNoiseMean, measNoiseCov] = ncs.controller.measModel.noise.getMeanAndCov();
            this.verifyEqual(measNoiseMean, zeros(dimV, 1));
            this.verifyEqual(measNoiseCov, this.configStruct.V);
        end
        
        %% testPlantDoublePendulumNoProcessNoise
        function testPlantDoublePendulumNoProcessNoise(this)
            dimPend = 6;            
            dimV = 3;
            
            expectedSamplingInterval = this.samplingInterval / 2;
            expectedPlantSamplingInterval = this.samplingInterval / 2;
            expectedWcontLin = blkdiag(0.1, 0.5, 0.2); % noise in the continuous-time linearization                      
            
            pendulum = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.1, 0.1, this.samplingInterval);
            pendulum.varDisturbanceForcePendulum1ContLin = expectedWcontLin(1, 1);
            pendulum.varDisturbanceForcePendulum2ContLin = expectedWcontLin(2, 2);
            pendulum.varDisturbanceForceActuatorContLin = expectedWcontLin(3, 3);
            pendulumCopyNoNoise = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.1, 0.1, this.samplingInterval);
            
            expectedPlantStateOrigin = zeros(dimPend, 1); % pendulum state is d-dimensional
            
            [A_d, B_d, C_d, ~] = pendulum.linearizeAroundUpwardEquilibrium(); % discrete time matrices
            [expA, expB, expC, ~] = pendulum.linearizeAroundUpwardEquilibrium(expectedSamplingInterval);
                                           
            this.configStruct.A = A_d;
            this.configStruct.B = B_d;
            this.configStruct.C = C_d;            
            this.configStruct.V = eye(dimV); % two-dimensional measurements
            this.configStruct.samplingInterval = expectedSamplingInterval;
            this.configStruct.plantSamplingInterval = expectedPlantSamplingInterval;
            this.configStruct.Q = eye(dimPend);
            this.configStruct.R = 100;
            this.configStruct.linearizationPoint = expectedPlantStateOrigin;
            this.configStruct.plant = pendulumCopyNoNoise;
            this.configStruct.W = expectedWcontLin; % inject the noise into the config here (for the linearization)
            this.configStruct.usePlantNoise = false; % plant to be simulated without noise
            this.configStruct.initialPlantState = zeros(dimPend, 1); % initial pendulum state
            
            this.configStruct.controllerClassName = 'NominalPredictiveController';
            this.configStruct.filterClassName = 'DelayedKF';
            this.configStruct.initialEstimate = Gaussian(zeros(dimPend, 1), eye(dimPend));
            expectedGain = -dlqr(expA, expB, this.configStruct.Q, this.configStruct.R);            
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);            
            this.verifyClass(ncs, ?NetworkedControlSystem);
            
            % check some params of the contructed NCS
            this.verifyEqual(ncs.networkType, NetworkType.UdpLikeWithAcks);            
            this.verifyEqual(ncs.name, this.id);
            this.verifyEqual(ncs.samplingInterval, expectedSamplingInterval);
            this.verifyEqual(ncs.plantSamplingInterval, expectedPlantSamplingInterval);
            
            this.verifyClass(ncs.controller, ?NcsControllerWithFilter);
            this.verifyClass(ncs.controller.controller, ?NominalPredictiveController);
            this.verifyEqual(ncs.controller.controller.L, expectedGain, 'AbsTol', 1e-8);
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength)
            
            % check if the filter uses the correct model (linearized dynamics) incl. noise
            [A_cont, ~, ~, G_cont, ~] = pendulum.linearizeAroundUpwardEquilibriumCont();
            expectedWdisc = integral(@(x) expm(A_cont*x) * G_cont * expectedWcontLin * G_cont' * expm(A_cont'*x), ...
                    0, expectedPlantSamplingInterval, 'ArrayValued', true);  
            this.verifyClass(ncs.controller.plantModel, ?LinearPlant);
            this.verifyEqual(ncs.controller.plantModel.sysMatrix, expA);
            this.verifyEqual(ncs.controller.plantModel.inputMatrix, expB);
            this.verifyClass(ncs.controller.plantModel.noise, ?Gaussian);
            [filterNoiseMean, filterNoiseCov] = ncs.controller.plantModel.noise.getMeanAndCov();
            this.verifyEqual(filterNoiseMean, zeros(dimPend,1));
            this.verifyEqual(filterNoiseCov, expectedWdisc, 'AbsTol', 1e-8);
            
            % plant is simulated without noise
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyClass(ncs.plant.plant, ?DoubleInvertedPendulum);
            this.verifyEqual(ncs.plant.plant.samplingInterval, expectedPlantSamplingInterval);
            this.verifyEmpty(ncs.plant.plant.noise);
            % buth the linearization has noise
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulum1ContLin, expectedWcontLin(1,1)); % noise affecting lower pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForcePendulum2ContLin, expectedWcontLin(2,2)); % noise affecting upper pendulum rod in the continuous-time linearization
            this.verifyEqual(ncs.plant.plant.varDisturbanceForceActuatorContLin, expectedWcontLin(3,3)); % noise affecting cart in the continuous-time linearization
                        
            % finally, check the measurement model assumed by the controller/filter
            this.verifyClass(ncs.controller.measModel, ?LinearMeasurementModel);
            this.verifyEqual(ncs.controller.measModel.measMatrix, expC);
            this.verifyClass(ncs.controller.measModel.noise, ?Gaussian);
            [measNoiseMean, measNoiseCov] = ncs.controller.measModel.noise.getMeanAndCov();
            this.verifyEqual(measNoiseMean, zeros(dimV, 1));
            this.verifyEqual(measNoiseCov, this.configStruct.V);
        end
%%        
%%        
        %% testNcsTranslatorFromFile
        function testNcsTranslatorFromFile(this)        
            % mat file is existing
            this.assertTrue(exist(this.translatorFilename, 'file') == 2);
            
            targetQoc = 0.75;
            expectedDataRate = targetQoc ^ 2 + 2 * targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
                        
            translatorFile = matfile(this.translatorFilename, 'Writable', true);
            translatorFile.translator = this.ncsTranslator;
            this.configStruct.translatorFile = this.translatorFilename;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);                        
            % check is translator is working properly
            [actualDataRate, actualRateChange] = ncs.evaluateRateQualityCharacteristics(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
        
        %% testNcsTranslatorFromConfig
        function testNcsTranslatorFromConfig(this)            
            targetQoc = 0.75;
            expectedDataRate = targetQoc ^ 2 + 2 * targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
                        
            this.configStruct.translator = this.ncsTranslator;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);                        
            % check is translator is working properly
            [actualDataRate, actualRateChange] = ncs.evaluateRateQualityCharacteristics(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
        
        %% testNcsTranslatorFromBoth
        function testNcsTranslatorFromBoth(this)
            % mat file is existing
            this.assertTrue(exist(this.translatorFilename, 'file') == 2);
            
            qocRateCurve = cfit(fittype('a*x^2+b*x+c'), 7, 2, 4);
            controlErrorQocCurve = cfit(fittype('-a*x+2'), 0.75); % for simplicity, use linear function
            maxDataRate = 55;
            secondTranslator = NcsTranslator(qocRateCurve, controlErrorQocCurve, maxDataRate);
            
            targetQoc = 0.75;
            expectedDataRate = targetQoc ^ 2 + 2 * targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
                        
            this.configStruct.translator = secondTranslator;
            translatorFile = matfile(this.translatorFilename, 'Writable', true);
            translatorFile.translator = this.ncsTranslator;
            this.configStruct.translatorFile = this.translatorFilename; % this one should be chosen
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);                        
            % check is translator is working properly
            [actualDataRate, actualRateChange] = ncs.evaluateRateQualityCharacteristics(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
    end
end

