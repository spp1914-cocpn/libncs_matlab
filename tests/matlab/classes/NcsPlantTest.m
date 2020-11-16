classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsPlantTest < matlab.unittest.TestCase
    % Test cases for NcsPlant.
    
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
            this.controlSeqLength = 3;
            this.maxSeqDelay = 3;
            
            this.A = eye(this.dimX);
            this.B = ones(this.dimX, this.dimU);
            this.W = eye(this.dimX); % sys noise cov
            this.sysModel = LinearPlant(this.A, this.B, this.W);
            this.actuator = BufferingActuator(this.controlSeqLength, zeros(this.dimU, 1));
            
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
            this.verifyEmpty(ncsPlant.getStatistics(1)); % the parameter passed here is irrelevant
        end
        
        %% testInitStatisticsRecordingNoInit
        function testInitStatisticsRecordingNoInit(this)
            expectedErrId = 'NcsPlant:InitStatisticsRecording';
            
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps * 10;
            this.verifyError(@()  this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps), ...
                expectedErrId);
        end
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps * 10;
            plantState = ones(this.dimX, 1);
            
            this.ncsPlantUnderTest.init(plantState);            
                        
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);
            
            % check the side effect, i.e., the proper creation and initialization of the structure to
            % store the data            
            actualStatistics = this.ncsPlantUnderTest.getStatistics(maxActuatorSteps);
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyEqual(actualStatistics.trueStates, [plantState nan(this.dimX, maxPlantSteps)]);
                        
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyEqual(actualStatistics.appliedInputs, nan(this.dimU, maxPlantSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyEqual(actualStatistics.trueModes, nan(1, maxActuatorSteps));
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyEqual(actualStatistics.numDiscardedControlSequences, nan(1, maxActuatorSteps));
        end
        
        %% testPlantStepNoInit
        function testPlantStepNoInit(this)
            expectedErrId = 'NcsPlant:PlantStep';
            
            plantTimestep = 1;
            this.verifyError(@() this.ncsPlantUnderTest.plantStep(plantTimestep), ...
                expectedErrId);
        end
        
        %% testPlantStep
        function testPlantStep(this)
            plantState = [2 3 4]';
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps * 10;
            
            rng(42);
            expectedPlantState = this.A * plantState + mvnrnd(zeros(3, 1), this.W)';
            
            this.ncsPlantUnderTest.init(plantState);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxActuatorSteps));
            
            rng(42); % should result in the same noise sample for the plant
            plantTimestep = 1;
            newPlantState = this.ncsPlantUnderTest.plantStep(plantTimestep);
            
            [storedPlantState, actualInput] = this.ncsPlantUnderTest.getPlantStatsForTimestep(plantTimestep);            
            
            this.verifyEqual(actualInput, zeros(this.dimU, 1));           
            this.verifyEqual(storedPlantState, newPlantState);
            this.verifyEqual(newPlantState, expectedPlantState);
        end
        
        %% testActuatorStepNoPackets
        function testActuatorStepNoPackets(this)
            plantState = [2 3 4]';
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps * 10; 
            caPackets = [];
            actuatorTimestep = 1;
            
            this.ncsPlantUnderTest.init(plantState);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxActuatorSteps));
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, caPackets); 
            [numDiscardedSeq, storedPlantMode] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
                        
            this.verifyEmpty(controllerAcks);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, this.controlSeqLength + 1);
            this.verifyEqual(storedPlantMode, this.controlSeqLength + 1);
        end
        
        %% testActuatorStep
        function testActuatorStep(this)
            plantState = [2 3 4]';
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps;             
            timestep = 1;
            actuatorTimestep = 2;
            plantTimestep = 2;
            id= 42;
            
            this.ncsPlantUnderTest.init(plantState);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxActuatorSteps));
            
            dataPacket = DataPacket(this.controlSequence, timestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            expectedMode = dataPacket.packetDelay + 1;
            expectedInput = this.controlSequence(:, expectedMode);
            rng(42);
            expectedPlantState = this.A * plantState + this.B * expectedInput + mvnrnd(zeros(3, 1), this.W)';
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, dataPacket);            
            [numDiscardedSeq, storedPlantMode] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
                        
            this.verifyNotEmpty(controllerAcks);
            this.verifyNumElements(controllerAcks, 1);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(storedPlantMode, expectedMode);
            
            % now we also perform a plant step
            rng(42); % should result in the same noise sample for the plant
            newPlantState = this.ncsPlantUnderTest.plantStep(plantTimestep);
            
            [storedPlantState, actualInput] = this.ncsPlantUnderTest.getPlantStatsForTimestep(plantTimestep);
                        
            this.verifyEqual(actualInput, expectedInput);
            this.verifyEqual(storedPlantState, newPlantState);
            this.verifyEqual(newPlantState, expectedPlantState);
        end
              
       
        %% testChangeActuatorSequenceLength
        function testChangeActuatorSequenceLength(this)
            plantState = [2 3 4]';
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps;             
            timestep = 1;
            actuatorTimestep = 2;
            id= 42;
            newSeqLength = 2;
            
            this.ncsPlantUnderTest.init(plantState);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxActuatorSteps));
            
            dataPacket = DataPacket(this.controlSequence(:, 1:newSeqLength), timestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, dataPacket);            
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
            expectedMode = dataPacket.packetDelay + 1;
           
            this.verifyNotEmpty(controllerAcks);
            this.verifyNumElements(controllerAcks, 1);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
        end
        
        %% testChangeActuatorSequenceLengthBufferedPacket
        function testChangeActuatorSequenceLengthBufferedPacket(this)
            plantState = [2 3 4]';
            maxActuatorSteps = 100;
            maxPlantSteps = maxActuatorSteps;     
            actuatorTimestep = 2;
                        
            this.ncsPlantUnderTest.init(plantState);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps, maxActuatorSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxActuatorSteps));
            
            id= 42;
            dataPacket = DataPacket(this.controlSequence, actuatorTimestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
                        
            % perform a step first so that packet is buffered
            [~, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep + dataPacket.packetDelay, dataPacket);
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep + dataPacket.packetDelay);
                        
            this.assertNotEmpty(controllerAcks); % assert that a packet is buffered
            this.assertEqual(numDiscardedSeq, 0);
            
            newSeqLength = 2;
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            caPackets = [];
            % then perform a step again
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep + dataPacket.packetDelay + 1, caPackets);   
                
            expectedMode = newSeqLength + 1;
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep + dataPacket.packetDelay + 1);
                                    
            this.verifyEmpty(controllerAcks); % we did not receive a packet from the controller
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
        end
        
        %% testIsStateAdmissible
        function testIsStateAdmissible(this)
            plantState = [2 3 4]';
            this.ncsPlantUnderTest.init(plantState);
            
            this.assertEmpty(this.sysModel.stateConstraints);            
            % no constraints given
            
            this.verifyTrue(this.ncsPlantUnderTest.isStateAdmissible());
            
            % set some constraints and check again
            this.sysModel.setStateConstraints([2 3 5], [7 8 9]);            
            this.assertNotEmpty(this.sysModel.stateConstraints);            
            
            this.verifyFalse(this.ncsPlantUnderTest.isStateAdmissible());
        end
    end
end

