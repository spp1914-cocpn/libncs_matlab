classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsInitializeTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_initialize.
    
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
        cacheLocation;
        matFilename;
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
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(~)
            ComponentMap.getInstance().clear();
            Cache.clear();
            % required to destroy the singleton instance
            clear ComponentMap;            
            clear Cache;
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
            this.caDelayProbs = [0.5 0.5];
            this.initialPlantState = zeros(this.dimX, 1); % plant state is already at the origin
            this.controllerClassName = 'NominalPredictiveController';
            
            import matlab.unittest.fixtures.WorkingFolderFixture;
            
            this.applyFixture(WorkingFolderFixture);
            this.cacheLocation = [pwd filesep 'cache'];
            this.matFilename = [pwd filesep 'test.mat'];
            
            configFile = matfile(this.matFilename, 'Writable', true);
            configFile.A = this.A;
            configFile.B = this.B;
            configFile.C = this.C;
            configFile.W = this.W;
            configFile.V = this.V;
            configFile.Q = this.Q;
            configFile.R = this.R;
            configFile.samplingInterval = this.samplingInterval;
            configFile.controlSequenceLength = this.controlSeqLength;
            configFile.maxMeasDelay = this.maxMeasDelay;
            configFile.maxControlSequenceDelay = this.maxControlSequenceDelay;
            
            this.configStruct.controllerClassName = this.controllerClassName;
            this.configStruct.initialPlantState = this.initialPlantState;
            this.configStruct.caDelayProbs = this.caDelayProbs;
            this.configStruct.ignoreControllerCache = false;
            
            Cache.setLocation(this.cacheLocation);
            
            this.dummyFile = [pwd filesep 'dummy.txt'];
            fid = fopen(this.dummyFile, 'w');
            fprintf(fid, 'dummy');
            fclose(fid);
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
        
        %% testInvalidMaxSimeTime
        function testInvalidMaxSimeTime(this)
            expectedErrId = 'ncs_initialize:InvalidMaxSimeTime';
            
            invalidMaxSimTime = 0; % not positive
            this.verifyError(@() ncs_initialize(invalidMaxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
            
            invalidMaxSimTime = eye(3); % not a scalar
            this.verifyError(@() ncs_initialize(invalidMaxSimTime, this.id, this.configStruct, this.matFilename), ...
                expectedErrId);
            
            invalidMaxSimTime = this; % not a scalar
            this.verifyError(@() ncs_initialize(invalidMaxSimTime, this.id, this.configStruct, this.matFilename), ...
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
                        
            invalidFile = this.cacheLocation; % not a file
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
        
        %% testInvalidLinearlyConstrainedPredictiveController
        function testInvalidLinearlyConstrainedPredictiveController(this)
            expectedErrId = 'ncs_initialize:InitController:LinearlyConstrainedPredictiveController';
            
             % we add constraint weightings, but no bounds
            this.configStruct.stateConstraintWeightings = ones(this.dimX, this.dimX, 2);
            this.configStruct.inputConstraintWeightings = ones(this.dimU, this.dimU, 2);
            this.configStruct.controllerClassName = 'LinearlyConstrainedPredictiveController';
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
        
        %% test
        function test(this)
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            this.verifyGreaterThan(ncsHandle, 0);
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
            
            this.verifyClass(ncs.sensor, ?NcsSensor);
            
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyEqual(ncs.plant.dimState, this.dimX);
            this.verifyEqual(ncs.plant.dimInput, this.dimU);
            
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
            
            % finally, check if the cacheinfo file was created
            [pathStr, configFileName, ~] = fileparts(this.matFilename);
            expectedFile = [pathStr filesep configFileName '.cacheinfo'];
            fileInfo = dir(expectedFile);
            this.verifyNotEmpty(fileInfo);
            this.verifyGreaterThan(fileInfo.bytes, 0);
            this.verifyFalse(fileInfo.isdir);
        end
        
        %% testNoCache
        function testNoCache(this)
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(zeros(this.dimX, 1), eye(this.dimX));            
            this.configStruct = rmfield(this.configStruct, 'ignoreControllerCache');
            
            this.assertFalse(isfield(this.configStruct, 'ignoreControllerCache'));
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            
            this.verifyGreaterThan(ncsHandle, 0);
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
            
            this.verifyClass(ncs.sensor, ?NcsSensor);
            
            this.verifyClass(ncs.plant, ?NcsPlant);
            this.verifyEqual(ncs.plant.dimState, this.dimX);
            this.verifyEqual(ncs.plant.dimInput, this.dimU);
            
            this.verifyEqual(ncs.controlSequenceLength, this.controlSeqLength);
            
            % finally, check that the cacheinfo file was indeed not created
            [pathStr, configFileName, ~] = fileparts(this.matFilename);
            expectedFile = [pathStr filesep configFileName '.cacheinfo'];
            fileInfo = dir(expectedFile);
            this.verifyEmpty(fileInfo);            
            this.verifyTrue(exist(expectedFile, 'file') == 0);
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
            caPackets = [];
            
            this.configStruct.filterClassName = 'DelayedModeIMMF';
            this.configStruct.initialEstimate = Gaussian(expectedMean, eye(this.dimX));
            this.configStruct.controllerEventBased = false; 
            this.configStruct.G = expectedSysNoiseMatrix;
            
            ncsHandle = ncs_initialize(this.maxSimTime, this.id, this.configStruct, this.matFilename);
            ncs = ComponentMap.getInstance().getComponent(ncsHandle);

            % no inputs applied
            [~, ~, newPlantState] = ncs.plant.step(timestep, caPackets, plantState);
          
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
    end
end

