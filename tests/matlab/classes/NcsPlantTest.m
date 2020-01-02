classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsPlantTest < matlab.unittest.TestCase
    % Test cases for NcsPlant.
    
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
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)
            maxLoopSteps = 100;
            plantState = ones(this.dimX, 1);
            plantMode = 2;
            
            this.ncsPlantUnderTest.initStatisticsRecording(maxLoopSteps, plantState, plantMode);
            
            % check the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = this.ncsPlantUnderTest.statistics;
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyEqual(actualStatistics.trueStates, [plantState nan(this.dimX, maxLoopSteps)]);
            
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyEqual(actualStatistics.trueModes, [plantMode nan(1, maxLoopSteps)]);
            
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyEqual(actualStatistics.appliedInputs, nan(this.dimU, maxLoopSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyEqual(actualStatistics.numDiscardedControlSequences, nan(1, maxLoopSteps));
        end
        
        %% testStepNoPackets
        function testStepNoPackets(this)
            plantState = [2 3 4]';
            timestep = 1;
            caPackets = [];
            
            this.ncsPlantUnderTest.initStatisticsRecording(10, plantState, 1);
                        
            [controllerAck, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep, caPackets, plantState);
            
            numDiscardedSeq = this.ncsPlantUnderTest.statistics.numDiscardedControlSequences(timestep);
            actualInput = this.ncsPlantUnderTest.statistics.appliedInputs(:, timestep);
            storedPlantMode = this.ncsPlantUnderTest.statistics.trueModes(timestep + 1);
            storedPlantState = this.ncsPlantUnderTest.statistics.trueStates(:, timestep + 1);
            
            this.verifyEmpty(controllerAck);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(actualInput, zeros(this.dimU, 1));
            this.verifyEqual(plantMode, this.controlSeqLength + 1);
            this.verifyEqual(storedPlantMode, this.controlSeqLength + 1);
            this.verifyEqual(storedPlantState, plantState);
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
            
            this.ncsPlantUnderTest.initStatisticsRecording(10, plantState, 1);
            
            caPackets = dataPacket;
            [controllerAck, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep + dataPacket.packetDelay, caPackets, plantState);
            
            expectedMode = dataPacket.packetDelay + 1;
            expectedInput = this.controlSequence(:, expectedMode);
            
            numDiscardedSeq = this.ncsPlantUnderTest.statistics.numDiscardedControlSequences(timestep + dataPacket.packetDelay);
            actualInput = this.ncsPlantUnderTest.statistics.appliedInputs(:, timestep + dataPacket.packetDelay);
            storedPlantMode = this.ncsPlantUnderTest.statistics.trueModes(timestep + 1 + dataPacket.packetDelay);
            storedPlantState = this.ncsPlantUnderTest.statistics.trueStates(:, timestep + 1 + dataPacket.packetDelay);
            
            this.verifyNotEmpty(controllerAck);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(storedPlantMode, expectedMode);
            this.verifyEqual(actualInput, expectedInput);
            this.verifyEqual(storedPlantState, plantState);
            this.verifyNotEmpty(newPlantState);
        end
       
        %% testChangeActuatorSequenceLength
        function testChangeActuatorSequenceLength(this)
            plantState = [2 3 4]';
            timestep = 2;
            id= 42;
            newSeqLength = 1;
            
            this.ncsPlantUnderTest.initStatisticsRecording(10, plantState, 1);
            
            dataPacket = DataPacket(this.controlSequence(:, 1:newSeqLength), timestep, id);
            dataPacket.packetDelay = 0;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            caPackets = dataPacket;
            
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            % then perform a step
            [controllerAck, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep + dataPacket.packetDelay, caPackets, plantState);
            
            expectedMode = dataPacket.packetDelay + 1;
            expectedInput = this.controlSequence(:, expectedMode);

            numDiscardedSeq = this.ncsPlantUnderTest.statistics.numDiscardedControlSequences(timestep + dataPacket.packetDelay);
            actualInput = this.ncsPlantUnderTest.statistics.appliedInputs(:, timestep + dataPacket.packetDelay);
            storedPlantMode = this.ncsPlantUnderTest.statistics.trueModes(timestep + 1 + dataPacket.packetDelay);
            storedPlantState = this.ncsPlantUnderTest.statistics.trueStates(:, timestep + 1 + dataPacket.packetDelay);
            
            this.verifyNotEmpty(controllerAck);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(storedPlantMode, expectedMode);
            this.verifyEqual(actualInput, expectedInput);
            this.verifyEqual(storedPlantState, plantState);
            this.verifyNotEmpty(newPlantState);
        end
        
        %% testChangeActuatorSequenceLengthBufferedPacket
        function testChangeActuatorSequenceLengthBufferedPacket(this)
            plantState = [2 3 4]';
            timestep = 2;
            id= 42;
            dataPacket = DataPacket(this.controlSequence, timestep, id);
            dataPacket.packetDelay = 0;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            
            this.ncsPlantUnderTest.initStatisticsRecording(10, plantState, 1);
            
            caPackets = dataPacket;
            % perform a step first so that packet is buffered
            [controllerAck, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep + dataPacket.packetDelay, caPackets, plantState);
            
            numDiscardedSeq = this.ncsPlantUnderTest.statistics.numDiscardedControlSequences(timestep + dataPacket.packetDelay);
            
            this.assertNotEmpty(controllerAck); % assert that a packet is buffered
            this.assertEqual(numDiscardedSeq, 0);
            
            newSeqLength = 1;
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            caPackets = [];
            % then perform a step again
            [controllerAck, plantMode, newPlantState] ...
                = this.ncsPlantUnderTest.step(timestep + dataPacket.packetDelay + 1, caPackets, plantState);
            
            expectedMode = newSeqLength + 1;
            expectedInput = zeros(this.dimU, 1); % the default input

            numDiscardedSeq = this.ncsPlantUnderTest.statistics.numDiscardedControlSequences(timestep + dataPacket.packetDelay + 1);
            actualInput = this.ncsPlantUnderTest.statistics.appliedInputs(:, timestep + dataPacket.packetDelay + 1);
            storedPlantMode = this.ncsPlantUnderTest.statistics.trueModes(timestep + 1 + dataPacket.packetDelay + 1);
            storedPlantState = this.ncsPlantUnderTest.statistics.trueStates(:, timestep + 1 + dataPacket.packetDelay + 1);
            
            this.verifyEmpty(controllerAck); % we did not receive a packet from the controller
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(storedPlantMode, expectedMode);
            this.verifyEqual(actualInput, expectedInput);
            this.verifyEqual(storedPlantState, plantState);
            this.verifyNotEmpty(newPlantState);
        end
        
        %% testIsStateAdmissible
        function testIsStateAdmissible(this)
            this.assertEmpty(this.sysModel.stateConstraints);
            plantState = [2 3 4]';
            % no constraints given
            this.verifyTrue(this.ncsPlantUnderTest.isStateAdmissible(plantState));
            
            % set some constraints and check again
            this.sysModel.setStateConstraints([2 3 5], [7 8 9]);
            this.assertNotEmpty(this.sysModel.stateConstraints);
            
            this.verifyFalse(this.ncsPlantUnderTest.isStateAdmissible(plantState));
        end
    end
end

