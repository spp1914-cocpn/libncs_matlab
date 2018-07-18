classdef EventBasedNcsSensor < NcsSensor
    % Wrapper class for (linear) sensors in an NCS that support event-based
    % data transmission in terms of the send-on-delta (e.g., deadband) strategy.
    %
    % Literature: 
    %  	Marek Miskowicz,
    %   Send-On-Delta Concept: An Event-Based Data Reporting Strategy,
    %   Sensors, vol. 6, no. 1, pp. 49-83, 2006.
    %
    %   Joris Sijs, Benjamin Noack, Mircea Lazar, Uwe D. Hanebeck,
    %   Event-Based Control and Signal Processing,
    %   Chapter 13 (Time-Periodic State Estimation with Event-Based Measurement Updates),
    %   CRC Press, Nov. 2015.
    
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
    
    properties (Access = private, Constant)
        defaultMeasurementDelta = 10;
    end
    
    properties (Access = public)
        % measurements delta (for the send-on-delta strategy)
        measurementDelta = EventBasedNcsSensor.defaultMeasurementDelta;
    end
    
    properties (Access = private)
        % previously sent measurement
        lastSentMeasurement;
    end
    
    methods
        function set.measurementDelta(this, newMeasurementDelta)
            assert(Checks.isNonNegativeScalar(newMeasurementDelta), ...
                'EventBasedNcsSensor:SetMeasurementDelta:InvalidDelta', ...
                '** <newMeasurementDelta> must be a nonnegative scalar **');

            this.measurementDelta = newMeasurementDelta;
        end
    end
    
    methods (Access = public)
        %% EventBasedNcsSensor
        function this = EventBasedNcsSensor(sensorModel, measurementDelta)
            % Class constructor.
            %
            % Parameters:
            %   >> sensorModel (LinearMeasurementModel instance)
            %      The measurement model to be utilized by the sensor.
            %
            %   >> measurementDelta (Nonnegative scalar)
            %      The measurement delta to be utilized for the send-on-delta-strategy.
            %
            % Returns:
            %   << this (EventBasedNcsSensor)
            %      A new EventBasedNcsSensor instance.
            
            this@NcsSensor(sensorModel);
            if nargin == 2
                this.measurementDelta = measurementDelta;
            end
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
            %      Empty matrix is returned in case none is to be transmitted (i.e., event has not occurred).
            %
            
            % take a measurement y_k
            measurement = this.sensor.simulate(plantState);

            % perform send-on-delta strategy
            if this.checkSendMeasurement(measurement)
                dataPacket = NcsSensor.createDataPacket(measurement, timestep);
                this.lastSentMeasurement = measurement;
            else
                % do not send the measurement
                dataPacket = [];
            end
        end
    end
    
    methods (Access = private)
        %% checkSendMeasurement
        function sendMeas = checkSendMeasurement(this, measurement)
            sendMeas = isempty(this.lastSentMeasurement) ...
                || norm(this.lastSentMeasurement - measurement) > this.measurementDelta;
        end
    end
end

