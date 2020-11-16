classdef NcsSensor < handle
    % Wrapper class for (linear) sensors in an NCS to provide a consistent
    % interface.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2020 Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (SetAccess = immutable, GetAccess = protected)
        sensor(1,1) LinearMeasurementModel;
    end
    
    properties(SetAccess = protected, GetAccess = public)
        % indicate whether sensor works event-based
        isEventBased(1,1) logical = false;
    end
    
    methods (Access = public)
        %% NcsSensor
        function this = NcsSensor(sensorModel)
            % Class constructor.
            %
            % Parameters:
            %   >> sensorModel (LinearMeasurementModel instance)
            %      The measurement model to be utilized by the sensor.
            %
            % Returns:
            %   << this (NcsSensor)
            %      A new NcsSensor instance.
            
            this.sensor = sensorModel;           
        end
        
        %% step
        function dataPacket = step(this, timestep, plantState)
            % Take a measurement of the plant state as part of a control cycle in an
            % NCS.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            %   >> plantState (Vector)
            %      The true state of the plant.
            %
            % Returns:
            %   << dataPacket (DataPacket or empty matrix)
            %      The data packet containing the measurement (as column vector) to be transmitted to the controller.
            %      Empty matrix is returned in case none is taken.
            %
            
            % take a measurement y_k and create packet
            dataPacket = NcsSensor.createDataPacket(this.sensor.simulate(plantState), timestep);
        end
    end
    
    methods (Access = protected, Static)
        %% createDataPacket
        function dataPacket = createDataPacket(measurement, timestep)
            % the measurement is transmitted from the sensor (id = 3) to the controller (id = 2)
            dataPacket = CreateDataPacket(measurement, timestep, 3, 2);
        end
    end
end

