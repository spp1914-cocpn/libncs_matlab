classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsPlantTest < matlab.unittest.TestCase
    % Test cases for NcsPlant.
    
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
            this.verifyEmpty(ncsPlant.getStatistics(1)); % the parameter here is irrelavant, since initStatistics() has not been called
        end
        
        %% testInitStatisticsRecordingNoInit
        function testInitStatisticsRecordingNoInit(this)
            expectedErrId = 'NcsPlant:InitStatisticsRecording';
            
            maxPlantSteps = 1000;
            % no initial plant state has been set
            this.verifyError(@()  this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps), ...
                expectedErrId);
        end
        
        %% testInitStatisticsRecording
        function testInitStatisticsRecording(this)            
            plantState = ones(this.dimX, 1);
            
            maxPlantSteps = 1000;            
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);            
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);
            
            % check the side effect, i.e., the proper creation and initialization of the structure to
            % store the data, everything should be empty but one true state recorded            
            actualStatistics = this.ncsPlantUnderTest.getStatistics(maxPlantSteps);
            this.verifyTrue(isfield(actualStatistics, 'trueStates'));
            this.verifyEqual(actualStatistics.trueStates, [plantState nan(this.dimX, maxPlantSteps)]);
                        
            this.verifyTrue(isfield(actualStatistics, 'appliedInputs'));
            this.verifyEqual(actualStatistics.appliedInputs, nan(this.dimU, maxPlantSteps));
                        
            this.verifyTrue(isfield(actualStatistics, 'trueModes'));
            this.verifyEmpty(actualStatistics.trueModes);
            
            this.verifyTrue(isfield(actualStatistics, 'numDiscardedControlSequences'));
            this.verifyEmpty(actualStatistics.numDiscardedControlSequences);
        end
        
        %% testPlantStepNoInit
        function testPlantStepNoInit(this)
            expectedErrId = 'NcsPlant:PlantStep';
            
            plantTimestep = 1;
            simTimeSec = 1;
            this.verifyError(@() this.ncsPlantUnderTest.plantStep(plantTimestep, simTimeSec), ...
                expectedErrId);
        end
        
        %% testPlantStep
        function testPlantStep(this)
            maxPlantSteps = 1000;
            plantState = [2 3 4]';            
            
            % draw a noise sample
            rng(42);
            noiseSample = this.sysModel.noise.drawRndSamples(1);
            expectedPlantState = this.A * plantState + noiseSample;
            
            rng(42); % should result in the same noise sample for the plant
            % plant noise samples are "precomputed" in init()
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
                        
            plantTimestep = 1;
            simTimeSec = 1; % for simplicity, 1 time step = 1 second
            newPlantState = this.ncsPlantUnderTest.plantStep(plantTimestep, simTimeSec);
            
            [storedPlantState, actualInput] = this.ncsPlantUnderTest.getPlantStatsForTimestep(plantTimestep);            
            
            this.verifyEqual(actualInput, zeros(this.dimU, 1));           
            this.verifyEqual(storedPlantState, newPlantState);
            this.verifyEqual(newPlantState, expectedPlantState);
        end
        
        %% testActuatorStepNoPackets
        function testActuatorStepNoPackets(this)
            maxPlantSteps = 1000;
            plantState = [2 3 4]';            
            caPackets = [];
            actuatorTimestep = 1;
            simTimeSec = 1; % for simplicity, 1 time step = 1 second
            
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, simTimeSec, caPackets); 
            [numDiscardedSeq, storedPlantMode] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
                        
            this.verifyEmpty(controllerAcks);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, this.controlSeqLength + 1);
            this.verifyEqual(storedPlantMode, this.controlSeqLength + 1);
        end
        
        %% testActuatorStep
        function testActuatorStep(this)
            maxPlantSteps = 2;
            plantState = [2 3 4]';          
            timestep = 1;
            actuatorTimestep = 2;
            plantTimestep = 2;
            simTimeSec = 2; % for simplicity, 1 time step = 1 second
            id = 42;
            
            rng(42); % seed before drawing noise samples in init()
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
            
            % perform a step to reach timestep 2            
            this.ncsPlantUnderTest.actuatorStep(1, 1, []);
            this.ncsPlantUnderTest.plantStep(1, 1);
            % propagate the expected state
            rng(42); % should result in the same two noise samples now
            noiseSamples = this.sysModel.noise.drawRndSamples(2);
            expectedPlantState = this.A * plantState + noiseSamples(:, 1);
            
            dataPacket = DataPacket(this.controlSequence, timestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            expectedMode = dataPacket.packetDelay + 1;
            expectedInput = this.controlSequence(:, expectedMode);            
            expectedPlantState = this.A * expectedPlantState + this.B * expectedInput + noiseSamples(:, 2);
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, simTimeSec, dataPacket);            
            [numDiscardedSeq, storedPlantMode] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
                        
            this.verifyNotEmpty(controllerAcks);
            this.verifyNumElements(controllerAcks, 1);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
            this.verifyEqual(storedPlantMode, expectedMode);
            
            % now we also perform a plant step            
            newPlantState = this.ncsPlantUnderTest.plantStep(plantTimestep, simTimeSec);
            
            [storedPlantState, actualInput] = this.ncsPlantUnderTest.getPlantStatsForTimestep(plantTimestep);
                        
            this.verifyEqual(actualInput, expectedInput);
            this.verifyEqual(storedPlantState, newPlantState, 'AbsTol', 1e-15);
            this.verifyEqual(newPlantState, expectedPlantState, 'AbsTol', 1e-15);
        end
%%
%%
        %% testGetInterpolatedPlantStateInvertedPendulum
        function testGetInterpolatedPlantStateInvertedPendulum(this)
            % we need a pendulum plant
            ta = 1; % 1 second
            pendulum = InvertedPendulum(1, 1, 1, 1, ta); % no noise            
            ncsPlant = NcsPlant(pendulum, BufferingActuator(2, 0));           
            
            maxPlantSteps = 10;
            plantState = [0 0 pi+0.001 0]'; % slightly away from equilibrium                       
   
            ncsPlant.init(plantState, maxPlantSteps);                       
            ncsPlant.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(ncsPlant.getStatistics(maxPlantSteps));
                        
            plantTimestep = 1;
            simTimeSec = 1; % for simplicity, 1 time step = 1 second
            % zero input       
            newPlantState = ncsPlant.plantStep(plantTimestep, simTimeSec);
                        
            zeroIntervalPortion = 0;
            interpolatedState = ncsPlant.getInterpolatedPlantState(plantTimestep, zeroIntervalPortion);
            
            this.verifyEqual(interpolatedState, newPlantState);
            
            nonzeroPortion = 0.5;
            interpolatedState = ncsPlant.getInterpolatedPlantState(plantTimestep, nonzeroPortion);
            % compute the expected input using a copy of the system with ta= 0.5
            pendCopy = InvertedPendulum(1, 1, 1, 1, ta * nonzeroPortion); % no noise
            expectedInterpolatedState = pendCopy.simulate(newPlantState); % no noise, no input
            
            this.verifyEqual(interpolatedState, expectedInterpolatedState);
            
        end
        
        %% testGetInterpolatedPlantStateDoubleInvertedPendulum
        function testGetInterpolatedPlantStateDoubleInvertedPendulum(this)
            % we need a pendulum plant
            ta = 1; % 1 second
            pendulum = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.75, 0.5, 0.5, ta); % no noise            
            ncsPlant = NcsPlant(pendulum, BufferingActuator(2, 0));           
            
            maxPlantSteps = 10;
            plantState = [0 0 0.001 0 0 0]'; % upper pendulum slightly away from equilibrium                       
   
            ncsPlant.init(plantState, maxPlantSteps);                       
            ncsPlant.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(ncsPlant.getStatistics(maxPlantSteps));
                        
            plantTimestep = 1;
            simTimeSec = 1; % for simplicity, 1 time step = 1 second
            % zero input       
            newPlantState = ncsPlant.plantStep(plantTimestep, simTimeSec);
                        
            zeroIntervalPortion = 0;
            interpolatedState = ncsPlant.getInterpolatedPlantState(plantTimestep, zeroIntervalPortion);
            
            this.verifyEqual(interpolatedState, newPlantState);
            
            nonzeroPortion = 0.5;
            interpolatedState = ncsPlant.getInterpolatedPlantState(plantTimestep, nonzeroPortion);
            % compute the expected input using a copy of the system with ta= 0.5
            pendCopy = DoubleInvertedPendulum(1, 1, 1, 1, 1, 0.75, 0.5, 0.5, ta * nonzeroPortion); % no noise  
            expectedInterpolatedState = pendCopy.simulate(newPlantState); % no noise, no input
            
            this.verifyEqual(interpolatedState, expectedInterpolatedState);
            
        end
        
        %% testGetInterpolatedPlantStateNoPendulum
        function testGetInterpolatedPlantStateNoPendulum(this)            
            maxPlantSteps = 10;
            plantState = [2 3 4]';            
            % plant noise samples are "precomputed" in init()
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
                        
            plantTimestep = 1;
            simTimeSec = 1; % for simplicity, 1 time step = 1 second
            newPlantState = this.ncsPlantUnderTest.plantStep(plantTimestep, simTimeSec);
            
            % always return last true state
            intervalPortion = 1/3;
            interpolatedState = this.ncsPlantUnderTest.getInterpolatedPlantState(plantTimestep, intervalPortion);
            
            this.verifyEqual(interpolatedState, newPlantState);
            
            % always return last true state
            intervalPortion = 1/7;
            interpolatedState = this.ncsPlantUnderTest.getInterpolatedPlantState(plantTimestep, intervalPortion);
            
            this.verifyEqual(interpolatedState, newPlantState);
        end
%%
%%
        %% testChangeActuatorSequenceLength
        function testChangeActuatorSequenceLength(this)
            maxPlantSteps = 1000;
            plantState = [2 3 4]';                      
            timestep = 1;
            actuatorTimestep = 2;
            simTimeSec = 2; % for simplicity, 1 time step = 1 second
            id= 42;
            newSeqLength = 2;
            
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
            
            % perform a step to reach timestep 2            
            this.ncsPlantUnderTest.actuatorStep(timestep, timestep, []);
            this.ncsPlantUnderTest.plantStep(timestep, timestep);
            
            dataPacket = DataPacket(this.controlSequence(:, 1:newSeqLength), timestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
            
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(actuatorTimestep, simTimeSec, dataPacket);            
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep);
            expectedMode = dataPacket.packetDelay + 1;
           
            this.verifyNotEmpty(controllerAcks);
            this.verifyNumElements(controllerAcks, 1);
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
        end
        
        %% testChangeActuatorSequenceLengthBufferedPacket
        function testChangeActuatorSequenceLengthBufferedPacket(this)
            maxPlantSteps = 1000;
            plantState = [2 3 4]';      
            actuatorTimestep = 2;
            simTimeSec = 2; % for simplicity, 1 time step = 1 second
                        
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);                       
            this.ncsPlantUnderTest.initStatisticsRecording(maxPlantSteps);            
            this.assertNotEmpty(this.ncsPlantUnderTest.getStatistics(maxPlantSteps));
            
            % perform two steps to reach timestep 3            
            this.ncsPlantUnderTest.actuatorStep(1, 1, []);
            this.ncsPlantUnderTest.plantStep(1, 1);
            this.ncsPlantUnderTest.actuatorStep(2, 2, []);
            this.ncsPlantUnderTest.plantStep(2, 2);
            
            id= 42;
            dataPacket = DataPacket(this.controlSequence, actuatorTimestep, id);
            dataPacket.packetDelay = 1;
            dataPacket.sourceAddress = 2;
            dataPacket.destinationAddress = 1;
                        
            % perform a step first (at time step 3) so that packet is buffered
            [~, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(...
                actuatorTimestep + dataPacket.packetDelay, simTimeSec + dataPacket.packetDelay, dataPacket);
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep + dataPacket.packetDelay);
                        
            this.assertNotEmpty(controllerAcks); % assert that a packet is buffered
            this.assertEqual(numDiscardedSeq, 0);
            
            newSeqLength = 2;
            % now change the sequence length
            this.ncsPlantUnderTest.changeActuatorSequenceLength(newSeqLength);
            caPackets = [];
            % then perform a step again
            [plantMode, controllerAcks] = this.ncsPlantUnderTest.actuatorStep(...
                actuatorTimestep + dataPacket.packetDelay + 1, simTimeSec + dataPacket.packetDelay + 1, caPackets);   
                
            expectedMode = newSeqLength + 1;
            [numDiscardedSeq, ~] = this.ncsPlantUnderTest.getActuatorStatsForTimestep(actuatorTimestep + dataPacket.packetDelay + 1);
                                    
            this.verifyEmpty(controllerAcks); % we did not receive a packet from the controller
            this.verifyEqual(numDiscardedSeq, 0);
            this.verifyEqual(plantMode, expectedMode);
        end
        
        %% testIsStateAdmissible
        function testIsStateAdmissible(this)
            maxPlantSteps = 10;
            plantState = [2 3 4]';
            this.ncsPlantUnderTest.init(plantState, maxPlantSteps);
            
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

