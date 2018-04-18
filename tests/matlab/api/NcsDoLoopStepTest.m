classdef NcsDoLoopStepTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_doLoopStep.
    
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

    properties (Access = private)
        ncs;
        ncsHandle;
        
        tickerInterval;
        
        componentMap;
        packetBuffer;
        
        timestamp;
        packetId;
        payload;
        scPacket;
        caPacket;
        
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
            this.packetBuffer.clearAll();
            % required to destroy the singleton instance
            clear ComponentMap;
            clear DataPacketBuffer;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.tickerInterval = 1; % 1s
            this.componentMap = ComponentMap.getInstance();
            this.packetBuffer = DataPacketBuffer.getInstance();
            
            this.maxLoopSteps = 10;
            this.maxMeasDelay = 2;
            this.controlSeqLength = 2;
            this.dimX = 3;
            this.dimU = 2;
            this.dimY = 1;
            
            % create a measurement packet
            this.timestamp = 2;
            this.payload = 42;
            this.packetId = 10;
            this.scPacket = DataPacket(this.payload, this.timestamp, this.packetId);
            this.scPacket.sourceAddress = 3; % from sensor
            this.scPacket.destinationAddress = 2; % to controller
         
            % create a control sequence packet
            controlSequence = ones(this.dimU, this.controlSeqLength);
            this.caPacket = DataPacket(controlSequence, this.timestamp, this.packetId + 1);
            this.caPacket.sourceAddress = 2; % from controller
            this.caPacket.destinationAddress = 1; % to actuator
            
            this.A = eye(this.dimX);
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
            this.filter = DelayedKF(this.maxMeasDelay);
            this.filter.setStateMeanAndCov(this.zeroPlantState, 0.5 * eye (this.dimX));
            
            this.ncs = NetworkedControlSystem('NCS', this.tickerInterval, NetworkType.TcpLike);
            this.ncs.plant = NcsPlant(LinearPlant(this.A, this.B, this.W), ...
                BufferingActuator(this.controlSeqLength, this.maxMeasDelay, zeros(this.dimU, 1)));
            this.ncs.controller = NcsControllerWithFilter(this.controller, this.filter, ...
                 DelayedKFSystemModel(this.A, this.B, Gaussian(zeros(this.dimX, 1), this.W), ...
                this.controlSeqLength + 1, this.maxMeasDelay, [1/3 1/3 1/3]), ...
                this.sensor, zeros(this.dimU, 1));
            this.ncs.sensor = NcsSensor(this.sensor, false);
            this.ncs.initPlant(this.zeroPlantState);
            
            this.ncs.initStatisticsRecording(this.maxLoopSteps);
            
            this.ncsHandle = this.componentMap.addComponent(this.ncs);
            % also, receive the packets
            this.packetBuffer.addPacket(this.ncsHandle, this.scPacket);
            this.packetBuffer.addPacket(this.ncsHandle, this.caPacket);
            
            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
       %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_doLoopStep(invalidHandle, this.timestamp), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 2; % not a valid index
            this.verifyError(@() ncs_doLoopStep(invalidHandle, this.timestamp), expectedErrId);
        end
        
        %% testInvalidTimestamp
        function testInvalidTimestamp(this)
            expectedErrId = 'ncs_doLoopStep:InvalidTimestamp';
            
            invalidTimestamp = this; % not a scalar
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, invalidTimestamp), expectedErrId);
            
            invalidTimestamp = 0; % integer, but not positive            
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, invalidTimestamp), expectedErrId);
            
            invalidTimestamp = 1.5; % positive, but fractional            
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, invalidTimestamp), expectedErrId);
        end
        
        %% testInvalidAcPacket
        function testInvalidAcPacket(this)
            invalidAcPacket = DataPacket([], this.timestamp, 42);
            invalidAcPacket.destinationAddress = 2; % to controller
            invalidAcPacket.sourceAddress = 1; % from actuator
            % receive the packet
            this.packetBuffer.addPacket(this.ncsHandle, invalidAcPacket);
            
            expectedErrId = 'ncs_doLoopStep:InvalidACKPacket';
            
            timestep = (this.timestamp + 1) * 1e12; % in pico-seconds
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, timestep), expectedErrId);
        end
        
        %% testInvalidDestinationAddress
        function testInvalidDestinationAddress(this)
            invalidPacket = DataPacket([], this.timestamp, 42);
            invalidPacket.destinationAddress = 42; % invalid
            invalidPacket.sourceAddress = 1; % from actuator
            % receive the packet
            this.packetBuffer.addPacket(this.ncsHandle, invalidPacket);
            
            expectedErrId = 'ncs_doLoopStep:InvalidDestinationAddress';
            
            timestep = (this.timestamp + 1) * 1e12; % in pico-seconds
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, timestep), expectedErrId);
        end
        
        %% testInvalidSourceAddressForControllerPacket
        function testInvalidSourceAddressForControllerPacket(this)
            invalidControllerPacket = DataPacket([], this.timestamp, 42);
            invalidControllerPacket.destinationAddress = 2; % to controller
            invalidControllerPacket.sourceAddress = 42; % from somewhere
            % receive the packet
            this.packetBuffer.addPacket(this.ncsHandle, invalidControllerPacket);
            
            expectedErrId = 'ncs_doLoopStep:InvalidDestinationAddress';
            
            timestep = (this.timestamp + 1) * 1e12; % in pico-seconds
            this.verifyError(@() ncs_doLoopStep(this.ncsHandle, timestep), expectedErrId);
        end
        
        %% test
        function test(this)
            import matlab.unittest.constraints.IsScalar
            
            timestep = (this.timestamp + 1) * 1e12; % in pico-seconds
            [pktsOut, stats] = ncs_doLoopStep(this.ncsHandle, timestep);
            
            % check if stats are as indicated
            this.verifyTrue(isstruct(stats));
            
            this.verifyTrue(isfield(stats, 'actual_qoc'));
            this.verifyThat(stats.actual_qoc, IsScalar);
            this.verifyGreaterThanOrEqual(stats.actual_qoc, 0);
            
            % one sc packet was received with delay 1
            this.verifyTrue(isfield(stats, 'sc_delays'));
            this.verifyThat(stats.sc_delays, IsScalar);
            this.verifyEqual(stats.sc_delays, 1);
            
            % one ca packet was received with delay 1
            this.verifyTrue(isfield(stats, 'ca_delays'));
            this.verifyThat(stats.ca_delays, IsScalar);
            this.verifyEqual(stats.ca_delays, 1);
            
            % no ca packet was received
            this.verifyTrue(isfield(stats, 'ac_delays'));
            this.verifyEmpty(stats.ac_delays);
            
            this.verifyTrue(isfield(stats, 'sc_sent'));
            this.verifyThat(stats.sc_sent, IsScalar);
            this.verifyClass(stats.sc_sent, ?logical);
            this.verifyTrue(stats.sc_sent);
            
            this.verifyTrue(isfield(stats, 'ca_sent'));
            this.verifyThat(stats.ca_sent, IsScalar);
            this.verifyClass(stats.ca_sent, ?logical);
            this.verifyTrue(stats.ca_sent);
            
            this.verifyTrue(isfield(stats, 'ac_sent'));
            this.verifyThat(stats.ac_sent, IsScalar);
            this.verifyClass(stats.ac_sent, ?logical);
            this.verifyFalse(stats.ac_sent);
            
            % we expect a cell array of data packets, column-vector like
            % with 2 elements, as network is TCP-like (no ACKs provided)
            this.verifyClass(pktsOut, ?cell);
            this.verifySize(pktsOut, [2 1]);
            this.verifyClass(pktsOut{1}, ?DataPacket);
            this.verifyClass(pktsOut{2}, ?DataPacket);
        end
    end
end

