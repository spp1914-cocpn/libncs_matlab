classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsDoHandleQocTargetTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_doHandleQocTarget.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2021 Florian Rosenthal <florian.rosenthal@kit.edu>
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
        eventBasedNcs;
        ncsHandle;
        eventBasedNcsHandle;
        
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
        
        qocTarget;
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
          
            this.maxMeasDelay = 2;
            this.controlSeqLength = 2;
            this.dimX = 4;
            this.dimU = 1;
            this.dimY = 2;            

            this.V = 0.1^2*eye(this.dimY); % variance of the meas noise
            this.Q = 2 * eye(this.dimX);
            this.R = 0.5 * eye(this.dimU);
            
            this.plant = InvertedPendulum(0.5, 0.3, 2, 0.1, this.plantTickerInterval);
            this.plant.varDisturbanceForcePendulumContLin = 1;
            this.plant.varDisturbanceForceActuatorContLin = 1;
            [this.A, this.B, this.C, this.W] = this.plant.linearizeAroundUpwardEquilibrium(this.tickerInterval);            
 
            this.zeroPlantState = zeros(this.dimX, 1); % plant state is already at the origin
            this.sensor = LinearMeasurementModel(this.C);
            this.sensor.setNoise(Gaussian(zeros(this.dimY, 1), this.V));
                       
            this.controller = NominalPredictiveController(this.A, this.B, this.Q, this.R, this.controlSeqLength);
            this.filter = DelayedKF(this.maxMeasDelay, eye(3));
            this.filter.setStateMeanAndCov(this.zeroPlantState, 0.5 * eye (this.dimX));
                       
            this.ncs = NetworkedControlSystem(NcsControllerWithFilter(this.controller, this.filter, ...
                    LinearPlant(this.A, this.B, this.W), this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4]'), ...
                NcsPlant(this.plant, BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1))), ...
                NcsSensor(this.sensor), 'NCS', this.tickerInterval, this.plantTickerInterval, NetworkType.TcpLike);
            
            eventBasedController = EventBasedNcsControllerWithFilter(this.controller, this.filter, ...
                    LinearPlant(this.A, this.B, this.W), this.sensor, zeros(this.dimU, 1), [1/4 1/4 1/4 1/4]');
            this.eventBasedNcs = NetworkedControlSystem(eventBasedController, ...
                    NcsPlant(this.plant, BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1))), ...
                    NcsSensor(this.sensor), 'NCS', this.tickerInterval, this.plantTickerInterval, NetworkType.TcpLike);           
            
            this.ncsHandle = this.componentMap.addComponent(this.ncs);
            this.eventBasedNcsHandle = this.componentMap.addComponent(this.eventBasedNcs);
            
            this.qocTarget = 0.7;
            
            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
        %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_doHandleQocTarget(invalidHandle, this.qocTarget), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 3; % not a valid index
            this.verifyError(@() ncs_doHandleQocTarget(invalidHandle, this.qocTarget), expectedErrId);
        end
        
        %% testEventBased
        function testEventBased(this)
            this.assertEqual(this.eventBasedNcs.controller.deadband, EventBasedNcsController.defaultDeadband);
            % check the case with event-based controller
            structOut = ncs_doHandleQocTarget(this.eventBasedNcsHandle, this.qocTarget);
            
            this.verifyEmpty(structOut); % nothing to be returned, empty struct
            this.verifyTrue(isstruct(structOut));
            deadband = this.eventBasedNcs.controller.deadband;
            % check the side effect, deadband value must have been changed
            this.verifyNotEqual(deadband, EventBasedNcsController.defaultDeadband);
            
            % now reduce qoc target again
            structOut = ncs_doHandleQocTarget(this.eventBasedNcsHandle, this.qocTarget / 2);
            newDeadband = this.eventBasedNcs.controller.deadband;
            this.verifyGreaterThan(newDeadband, deadband); % deadband should have increased
        end
        
        %% testSamplingRate
        function testSamplingRate(this)
            import matlab.unittest.constraints.IsScalar
            import matlab.unittest.constraints.IsGreaterThanOrEqualTo           
            
            % attach a translator first
            qualityRateCurve = cfit(fittype('a*x'), 100); % simple, linear relationship
            controlErrorQualityCurve = cfit(fittype('a*x'), 1); % directly map error to QoC
            maxRate = 200;
            translator = NcsTranslator(qualityRateCurve, controlErrorQualityCurve, maxRate);
            this.ncs.attachTranslator(translator);
            % changing of sampling rate is so far only supported for pendulum
            % sampling interval is returned in pico-seconds
            structOut = ncs_doHandleQocTarget(this.ncsHandle, this.qocTarget);
            expectedNewSamplingInterval = 1e12 / qualityRateCurve(this.qocTarget);
            
            this.verifyNotEmpty(structOut);
            this.verifyTrue(isstruct(structOut));
            
            % we expect a struct with a two fields
            this.verifyEqual(numel(fieldnames(structOut)), 2);
            this.verifyTrue(isfield(structOut, 'samplingInterval'));
            this.verifyTrue(isfield(structOut, 'deviationFactor'));
            
            % must be larger then 1/maxRate
            this.verifyThat(structOut.samplingInterval, IsGreaterThanOrEqualTo(1e12/maxRate));
            this.verifyEqual(structOut.samplingInterval, expectedNewSamplingInterval);
            this.verifyEqual(structOut.deviationFactor, 1);
            
            % now reduce qoc target again
            structOut = ncs_doHandleQocTarget(this.ncsHandle, this.qocTarget / 2);
            expectedNewSamplingInterval = 1e12 / qualityRateCurve(this.qocTarget / 2);
            this.verifyEqual(structOut.samplingInterval, expectedNewSamplingInterval);
            this.verifyEqual(structOut.deviationFactor, 1);
        end
    end
end

