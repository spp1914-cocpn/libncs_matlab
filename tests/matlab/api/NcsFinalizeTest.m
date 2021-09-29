classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsFinalizeTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_finalize.
    
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
        ncs;
        ncsHandle;
        
        tickerInterval;
        plantTickerInterval;
        
        componentMap;
        
        maxLoopSteps;
        maxPlantSteps;
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
        zeroPlantState;

        actuator;
        controller;
        filter;
        sensor;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(this)
            this.componentMap.clear();
            % required to destroy the singleton instance
            clear ComponentMap;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.tickerInterval = 1; % 1s
            this.plantTickerInterval = 1;
            this.componentMap = ComponentMap.getInstance();
            
            this.maxLoopSteps = 1;
            this.maxPlantSteps = 1;
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
            this.zeroPlantState = zeros(this.dimX, 1); % plant state is already at the origin
            this.sensor = LinearMeasurementModel(this.C);
            this.sensor.setNoise(Gaussian(0, this.V));
                       
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.filter = DelayedKF(this.maxMeasDelay, eye(3));
            this.filter.setStateMeanAndCov(this.zeroPlantState, 0.5 * eye (this.dimX));            
            
            ncsPlant = NcsPlant(LinearPlant(this.A, this.B, this.W), ...
                BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1)));
            ncsController = NcsControllerWithFilter(this.controller, this.filter, ...
                 LinearPlant(this.A, this.B, this.W), this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4 1/4]);
            ncsSensor = NcsSensor(this.sensor);
            
            this.ncs = NetworkedControlSystem(ncsController, ncsPlant, ncsSensor, ...
                'NCS', this.tickerInterval, this.plantTickerInterval, NetworkType.TcpLike);
            
            % init plant
            maxSimTime = ConvertToPicoseconds(this.maxPlantSteps * this.plantTickerInterval);
            this.ncs.initPlant(this.zeroPlantState, maxSimTime);
            this.ncs.initStatisticsRecording(maxSimTime);
            
            % setup the translator
            qocRateCurve = cfit(fittype('a/x'), 1);
            controlErrorQocCurve = cfit(fittype('a*x^3'), 1.5);            
            this.ncs.attachTranslator(NcsTranslator(qocRateCurve, controlErrorQocCurve, 1 / this.tickerInterval));            
            
            this.ncsHandle = this.componentMap.addComponent(this.ncs);
            
            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
        %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_finalize(invalidHandle), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 2; % not a valid index
            this.verifyError(@() ncs_finalize(invalidHandle), expectedErrId);
        end
        
        %% test
        function test(this)
            import matlab.unittest.constraints.IsScalar
            this.assertTrue(this.componentMap.containsComponent(this.ncs));
            
            % perform a control cycle
            simTime = this.maxLoopSteps * 1e12; % in pico-seconds
            ncs_doPlantStep(this.ncsHandle, simTime);
            ncs_doLoopStep(this.ncsHandle, simTime);
            
            [costs, controllerStats, plantStats] = ncs_finalize(this.ncsHandle);
            
            % check if stats are as indicated
            this.verifyTrue(isstruct(controllerStats));
            this.verifyTrue(isstruct(plantStats));
            
            this.verifyTrue(isfield(controllerStats, 'numUsedMeasurements'));
            this.verifySize(controllerStats.numUsedMeasurements, [1 this.maxLoopSteps]);
            this.verifyGreaterThanOrEqual(controllerStats.numUsedMeasurements, 0);
             
            this.verifyTrue(isfield(controllerStats, 'numDiscardedMeasurements'));
            this.verifySize(controllerStats.numDiscardedMeasurements, [1 this.maxLoopSteps]);
            this.verifyGreaterThanOrEqual(controllerStats.numDiscardedMeasurements, 0);
                        
            this.verifyTrue(isfield(controllerStats, 'controllerStates'));
            this.verifySize(controllerStats.controllerStates, [this.dimX this.maxLoopSteps + 1]);
            
            this.verifyTrue(isfield(controllerStats, 'controllerTimes'));
            this.verifySize(controllerStats.controllerTimes, [1 this.maxLoopSteps + 1]); % row vector
            
            this.verifyTrue(isfield(controllerStats, 'numDiscardedControlSequences'));
            this.verifySize(controllerStats.numDiscardedControlSequences, [1 this.maxLoopSteps]);
            this.verifyGreaterThanOrEqual(controllerStats.numDiscardedControlSequences, 0);
            
            this.verifyTrue(isfield(plantStats, 'appliedInputs'));
            this.verifySize(plantStats.appliedInputs, [this.dimU this.maxPlantSteps]);
            
            this.verifyTrue(isfield(plantStats, 'trueStates'));
            this.verifySize(plantStats.trueStates, [this.dimX this.maxPlantSteps + 1]);
            
            % now check the returned value of the cost function
            this.verifyThat(costs, IsScalar);
            this.verifyGreaterThanOrEqual(costs, 0);
            
            this.verifyFalse(this.componentMap.containsComponent(this.ncs));
         end
    end
end

