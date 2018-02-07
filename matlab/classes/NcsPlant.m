classdef NcsPlant < handle
    % Wrapper class for (linear) subsystem actuator/plant in an NCS to provide a consistent
    % interface.
    
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
    
    properties(SetAccess = immutable, GetAccess = protected)
         plant@LinearPlant;
         actuator@BufferingActuator;
    end
    
    properties (GetAccess = public, Dependent)
        dimState;
        dimInput;
    end
    
    methods
        function dim = get.dimState(this)
            dim = this.plant.dimState;
        end
        
        function dim = get.dimInput(this)
            dim = this.actuator.dimU;
        end
    end
    
    methods (Access = public)
        %% NcsPlant
        function this = NcsPlant(plant, actuator)
            % Class constructor.
            %
            % Parameters:
            %   >> plant (LinearPlant instance)
            %      The plant to be controlled.
            %
            %   >> actuator (BufferingActuator instance)
            %      The actuator in the NCS which applies inputs to the plant.
            %
            % Returns:
            %   << this (NcsSensor)
            %      A new NcsPlant instance.
            
            this.plant = plant;
            this.actuator = actuator;
        end
        
        %% step
        function [controllerAck, numDiscardedSeq, actualInput, plantMode, newPlantState] ...
                = step(this, timestep, caPackets, plantState)
            % Process received packets (i.e., control sequences) from the controller and apply an appropriate input u_k
            % as part of a control cycle in an NCS.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            %   >> caPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing control sequences transmitted from the controller.
            %
            %   >> plantState (Vector)
            %      The true state of the plant.
            %
            % Returns:
            %   << controllerAck (Column vector, might be empty)
            %      In case any of the received control sequences replaces
            %      the one currently buffered by the actuator, an ACK
            %      packet for the corresponding data packet is created.
            %      Empty matrix is returned in case none became active.
            %
            %   << numDiscardedSeq (Nonnegative integer)
            %      The number of received control sequences that were
            %      discarded in this step. Possible values are
            %      numel(caPackets) and numel(caPackets) - 1.
            %
            %   << actualInput (Column vector)
            %      The buffered control input to be applied at the given time step (u_k), if
            %      present. If not, then the default input is applied and
            %      returned here.
            %
            %   << plantMode (Positive integer)
            %      The mode of the underlying MJLS that corresponds to the
            %      applied input (i.e., the age of the buffered sequence), i.e., theta_k.
            %
            %   << newPlantState (Column vector)
            %      The plant state after the input has been applied, i.e., x_{k+1}.
            
            controllerAck = this.actuator.processControllerPackets(caPackets);
            if isempty(controllerAck)
                numDiscardedSeq = numel(caPackets);
            else
                numDiscardedSeq = numel(caPackets) - 1;
            end
                        
            % get actual input u_k and theta_k
            [actualInput, plantMode] = this.actuator.getCurrentInput(timestep);
            % apply the input to proceed to the next time step
            this.plant.setSystemInput(actualInput);
            newPlantState = this.plant.simulate(plantState);
        end
    end
    
end

