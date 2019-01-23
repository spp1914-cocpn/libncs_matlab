classdef EventBasedControllerTriggerCriterion < uint8
    % Enum to provide constants indicating what kind of event-trigger 
    % to be used by the controller for transmission of control sequences.
        
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
    
    enumeration
        QoC (1), % checks only the deviation in the QoC
        StageCosts (2), % considers the stage costs
        Sequence (3) % checks for squared deviations in the sequences only
    end
    
    properties
    end
    
    methods (Access = public)
        %% evaluateTrigger
        function ret = evaluateTrigger(this, lastSentData, sendData, deadband)
            switch this
                case EventBasedControllerTriggerCriterion.Sequence
                    % extract the relevant inputs from the two sequences (which
                    % are given as matrices)
                    timeDiff = sendData.timestep - lastSentData.timestep;
                    newU = sendData.sequence(:, 1:(end-timeDiff));
                    oldU = lastSentData.sequence(:, timeDiff+1:end);
                   
                    maxChange = max(sqrt(sum((newU-oldU).^2, 1)) ./ sqrt(sum(newU.^2, 1)));
                    % vecnorm function available with R2017b
                    ret = maxChange > deadband;
                case EventBasedControllerTriggerCriterion.QoC
                    ret = abs(sendData.qoc - lastSentData.qoc) > deadband;
                case EventBasedControllerTriggerCriterion.StageCosts
                    % check the deviation of the stage costs
                    ret = abs(sendData.stageCosts - lastSentData.stageCosts) > deadband;
            end
        end
    end
    
    methods (Access = public, Static)
        %% getMaxIdx
        function maxId = getMaxId()
            maxId = 3;
        end
    end
end

