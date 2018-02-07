classdef NcsPlantTest < matlab.unittest.TestCase
    % Test cases for NcsPlant.
    
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
    
    properties
        dimX;
        dimU;
        controlSeqLength;
        maxSeqDelay;
        
        A;
        B;
        W;
        
        actuator;
        sysModel;
        
        controlSequence;
        dimSequence;
        
        ncsPlantUnderTest;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.dimX = 3;
            this.dimU = 2;
            this.controlSeqLength = 2;
            this.maxSeqDelay = 2;
            
            this.A = eye(this.dimX);
            this.B = ones(this.dimX, this.dimU);
            this.W = eye(this.dimX); % sys noise cov
            this.sysModel = LinearPlant(this.A, this.B, this.W);
            this.actuator = BufferingActuator(this.controlSeqLength, this.maxSeqDelay, zeros(this.dimU, 1));
            
            this.dimSequence = this.dimU * this.controlSeqLength;
            maxValue = max(this.dimU, this.controlSeqLength);
            tmp = gallery('moler', maxValue);
            this.controlSequence = tmp(1:this.dimU, 1:this.controlSeqLength);
            
            this.ncsPlantUnderTest = NcsPlant(this.sysModel, this.actuator);
        end
    end
    
    methods (Test)
        %% testNcsPlant
        function testNcsPlant(this)
            ncsPlant = NcsPlant(this.sysModel, this.actuator);
            
            this.verifyEqual(ncsPlant.dimState, this.dimX);
            this.verifyEqual(ncsPlant.dimInput, this.dimU);
        end
        
        %% testStepNoPackets
        function testStepNoPackets(this)
            plantState = [2 3 4]';
            timestep = 1;
            caPackets = [];
            
            [controllerAck, numDiscardedSeq, actualInput, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep, caPackets, plantState);
            
            this.verifyEmpty(controllerAck);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(actualInput, zeros(this.dimU, 1));
            this.verifyEqual(plantMode, this.controlSeqLength + 1);
            this.verifyNotEmpty(newPlantState);
        end
        
        %% testStep
        function testStep(this)
            plantState = [2 3 4]';
            timestep = 2;
            id= 42;
            dataPacket = DataPacket(this.controlSequence, timestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            
            caPackets = dataPacket;
            [controllerAck, numDiscardedSeq, actualInput, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep + dataPacket.packetDelay, caPackets, plantState);
            
            expectedMode = dataPacket.packetDelay + 1;
            expectedInput = this.controlSequence(:, expectedMode);
            
            this.verifyNotEmpty(controllerAck);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(actualInput, expectedInput);
            this.verifyNotEmpty(newPlantState);
       end
    end
end

