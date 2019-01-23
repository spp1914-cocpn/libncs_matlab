classdef EventBasedControllerTriggerCriterionTest < matlab.unittest.TestCase
    % Test cases for EventBasedControllerTriggerCriterion.
    
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
        dimU;
        sequenceLength;
        
        lastSentSequence;   
        lastSentTimestep;
        lastSentQoc;
        lastSentStageCosts;
        lastSentData;
        deadband;
    end
    
    methods (TestMethodSetup)
         %% init
        function init(this)
            this.dimU = 2;
            this.sequenceLength = 3;
            this.deadband = 0.5;
            
            this.lastSentSequence = ones(this.dimU, this.sequenceLength);
            this.lastSentQoc = 2;
            this.lastSentStageCosts = 4;
            this.lastSentTimestep = 1;
           
            this.lastSentData.sequence = this.lastSentSequence;
            this.lastSentData.qoc = this.lastSentQoc;
            this.lastSentData.stageCosts = this.lastSentStageCosts;
            this.lastSentData.timestep = this.lastSentTimestep;
        end
    end
    
    methods (Test)
        %% testEvaluateTriggerQoC
        function testEvaluateTriggerQoC(this)
            criterionUnderTest = EventBasedControllerTriggerCriterion.QoC;
            
            % we do not send
            sendData.qoc = this.deadband -1 + this.lastSentQoc;            
            this.verifyFalse(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
            
            % we send
            sendData.qoc = this.deadband + this.lastSentQoc + 1;            
            this.verifyTrue(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
        end
        
        %% testEvaluateTriggerStageCosts
        function testEvaluateTriggerStageCosts(this)
            criterionUnderTest = EventBasedControllerTriggerCriterion.StageCosts;
            
            % we do not send
            sendData.stageCosts = this.lastSentStageCosts;            
            this.verifyFalse(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
            
            % we send
            sendData.stageCosts = -2 * this.lastSentStageCosts;            
            this.verifyTrue(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
        end
        
        %% testEvaluateTriggerSequence
        function testEvaluateTriggerSequence(this)
            criterionUnderTest = EventBasedControllerTriggerCriterion.Sequence;
            
            % we do not send
            sendData.sequence = this.lastSentSequence;            
            sendData.timestep = this.lastSentTimestep + 1;
            this.verifyFalse(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
            
            % we send
            sendData.sequence = 1000 * this.lastSentSequence;            
            sendData.timestep = this.lastSentTimestep + 1;           
            this.verifyTrue(criterionUnderTest.evaluateTrigger(this.lastSentData, sendData, this.deadband));
        end
        
        %% testGetMaxId
        function testGetMaxId(this)
            expectedId = 3;
            
            this.verifyEqual(EventBasedControllerTriggerCriterion.getMaxId(), expectedId);
        end
    end
    
end

