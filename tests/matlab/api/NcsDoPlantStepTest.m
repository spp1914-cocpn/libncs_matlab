classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsDoPlantStepTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_doPlantStep.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2020  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        
        timestamp;
                
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
            this.plantTickerInterval = 1; % 1s 
            this.componentMap = ComponentMap.getInstance();
            this.timestamp = 2;
            
            this.maxLoopSteps = 10;
            this.maxPlantSteps = 10;
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
                        
            this.ncs = NetworkedControlSystem(NcsControllerWithFilter(this.controller, this.filter, ...
                    DelayedKFSystemModel(this.A, this.B, Gaussian(zeros(this.dimX, 1), this.W), ...
                    this.controlSeqLength + 1, this.maxMeasDelay, [1/3 1/3 1/3]), ...
                    this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4]'), ...
                NcsPlant(this.plant, BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1))), ...
                NcsSensor(this.sensor), 'NCS', this.tickerInterval, this.plantTickerInterval, NetworkType.TcpLike);
            this.ncs.initPlant(this.zeroPlantState);
            
            maxSimTime = ConvertToPicoseconds(this.maxPlantSteps * this.plantTickerInterval);
            this.ncs.initStatisticsRecording(maxSimTime);
            
            this.ncsHandle = this.componentMap.addComponent(this.ncs);            
            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
       %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_doPlantStep(invalidHandle, this.timestamp), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 2; % not a valid index
            this.verifyError(@() ncs_doPlantStep(invalidHandle, this.timestamp), expectedErrId);
        end
    
        %% test
        function test(this)
            rng(42);
            timestep = (this.timestamp + 1) * 1e12; % in pico-seconds
            plantStateAdmissible = ncs_doPlantStep(this.ncsHandle, uint64(timestep));            
                        
            import matlab.unittest.constraints.IsScalar
            import matlab.unittest.constraints.IsOfClass                        
            
            this.verifyThat(plantStateAdmissible, IsScalar);
            this.verifyThat(plantStateAdmissible, IsOfClass(?logical));
            this.verifyTrue(plantStateAdmissible);
            
            % now set some constraints, so that the plant state becomes invalid
            this.plant.setStateConstraints([-inf -inf -inf], [-1000 -1000 -1000]);
            this.assertEqual(this.plant.stateConstraints(:), [-inf; -inf; -inf; -1000; -1000; -1000]);
            
            timestep = (this.timestamp + 2) * 1e12; % in pico-seconds            
            plantStateAdmissible = ncs_doPlantStep(this.ncsHandle, uint64(timestep));
            
            this.verifyThat(plantStateAdmissible, IsScalar);
            this.verifyThat(plantStateAdmissible, IsOfClass(?logical));
            this.verifyFalse(plantStateAdmissible);
        end
    end
end

