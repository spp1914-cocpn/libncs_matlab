classdef NcsSensor < handle
    % Wrapper class for (linear) sensors in an NCS to provide a consistent
    % interface.
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    %      
    %   Markus Jung, Florian Rosenthal, and Martina Zitterbart,
    %   CoCPN-Sim: An Integrated Simulation Environment for Cyber-Physical Systems,
    %   Proceedings of the 2018 IEEE/ACM Third International Conference on Internet-of-Things Design and Implementation (IoTDI), 
    %   Orlando, FL, USA, April 2018.
    
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
    
    properties (SetAccess = immutable, GetAccess = protected)
        sensor % (1,1) AdditiveNoiseMeasurementModel
        sensorNoise; % for repeatability, draw in advance
    end
    
    properties(SetAccess = protected, GetAccess = public)
        % indicate whether sensor works event-based
        isEventBased(1,1) logical = false;
    end
        
    methods (Access = public)
        %% NcsSensor
        function this = NcsSensor(sensorModel, maxSensorSteps)
            % Class constructor.
            %
            % Parameters:
            %   >> sensorModel (AdditiveNoiseMeasurementModel instance)
            %      The measurement model to be utilized by the sensor.
            %
            %   >> maxSensorSteps (Positive integer)
            %      A positive integer denoting the maximum number of sensor invocations to be carried out during the simulation.
            %
            %      Note: The actual number of sensor invocations/steps carried out during a
            %      simulation can be smaller than specified by this parameter because an NCS can be finished
            %      prematurely (e.g., due to errors at runtime), event-based communication, varying sampling rates of controller and sensor,
            %      or a parameter in an Omnet ini-file indicating that an NCS should not be
            %      active over the whole simulation time.
            %
            % Returns:
            %   << this (NcsSensor)
            %      A new NcsSensor instance.

            arguments
               sensorModel (1,1) AdditiveNoiseMeasurementModel
               maxSensorSteps (1,1) double {mustBePositive, mustBeInteger}
            end
            
            this.sensor = sensorModel;           
            % draw all sensor noises in advance, for repeatability
            this.sensorNoise = this.sensor.noise.drawRndSamples(maxSensorSteps);
        end
        
        %% step
        function dataPacket = step(this, timestep, plantState)
            % Take a measurement of the plant state as part of a control cycle in an
            % NCS.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, the integer yielding the
            %      current simulation time (in seconds) when multiplied by the
            %      loop's sampling interval in case the sampling interval is fixed.
            %
            %   >> plantState (Vector)
            %      The true state of the plant.
            %
            % Returns:
            %   << dataPacket (DataPacket or empty matrix)
            %      The data packet containing the measurement (as column vector) to be transmitted to the controller.
            %      Empty matrix is returned in case none is taken.
            
            % take a measurement y_k and create packet
            dataPacket = NcsSensor.createDataPacket(...
                this.sensor.measurementEquation(plantState) + this.sensorNoise(:, timestep), timestep);
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

