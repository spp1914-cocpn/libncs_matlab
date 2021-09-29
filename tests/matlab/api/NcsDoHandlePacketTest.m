classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsDoHandlePacketTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_doHandlePacket.
    
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
        ncsHandle;
        tickerInterval;
        
        timestamp;
        packetId;
        payload;
        dataPacket;
        
        componentMap;
        packetBuffer;
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
            maxMeasDelay = 2;
            controlSeqLength = 2;
            
            dimX = 3;
            dimU = 2;
            
            A = 0.75 * eye(dimX);
            B = ones(dimX, dimU);
            C = [1 2 3];
            W = eye(dimX); % sys noise cov
            V = 0.1^2; % variance of the meas noise
            Q = 2 * eye(dimX);
            R = 0.5 * eye(dimU);
            plant = LinearPlant(A, B, W);
            sensor = LinearMeasurementModel(C);
            sensor.setNoise(Gaussian(0, V));
            sensorSubsystem = NcsSensor(sensor);
            
            filter = DelayedKF(maxMeasDelay, eye(3));
            filterPlantModel = LinearPlant(A, B, W);
            
            actuator = BufferingActuator(controlSeqLength, zeros(dimU, 1));
            plantSubsystem = NcsPlant(plant, actuator);
            
            controller = NominalPredictiveController(A, B, Q, R, controlSeqLength);
            controllerSubsystem = NcsControllerWithFilter(controller, filter, ...
                filterPlantModel, sensor, zeros(dimU, 1), [1/4 1/4 1/4 1/4]');            
            
            this.componentMap = ComponentMap.getInstance();    
            this.packetBuffer = DataPacketBuffer.getInstance();
            this.tickerInterval = .2; % 0.2s
            this.ncs = NetworkedControlSystem(controllerSubsystem, plantSubsystem, sensorSubsystem, ...
                'NCS', this.tickerInterval, NetworkType.TcpLike);
            this.ncsHandle = this.componentMap.addComponent(this.ncs);
            
            this.timestamp = 2;
            this.payload = 42;
            this.packetId = 10;
            this.dataPacket = DataPacket(this.payload, this.timestamp, this.packetId);
            
            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
        %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_doHandlePacket(invalidHandle, this.timestamp, this.dataPacket), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 2; % not a valid index
            this.verifyError(@() ncs_doHandlePacket(invalidHandle, this.timestamp, this.dataPacket), expectedErrId);
        end        
        
        %% test
        function test(this)
            this.assertEmpty(this.packetBuffer.getDataPackets(this.ncsHandle));
            
            packetsOut = ncs_doHandlePacket(this.ncsHandle, this.timestamp, this.dataPacket);
            bufferedPackets = this.packetBuffer.getDataPackets(this.ncsHandle);
            
            % no packets shall be returned
            this.verifyEmpty(packetsOut);
            % validate the side effect: packet should be stored in buffer
            this.verifySize(bufferedPackets, [1 1]);
            this.verifyEqual(bufferedPackets(1), this.dataPacket);
        end
    end
end

