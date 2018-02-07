classdef NcsSensor < handle
    % Wrapper class for (linear) sensors in an NCS to provide a consistent
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
    
    properties (SetAccess = immutable, GetAccess = protected)
        measModel@LinearMeasurementModel;
    end
    
    properties (Access = private)
        % previously sent measurement
        lastSentMeasurement;
        % measurements delta (for the send-on-delta strategy)
        % disable send-on-delta by default
        measurementDelta = -1;%5;
    end
    
    methods (Access = public)
        %% NcsSensor
        function this = NcsSensor(measModel)
            % Class constructor.
            %
            % Parameters:
            %   >> measModel (LinearMeasurementModel instance)
            %      The measurement model to be utilized by the sensor.
            %
            % Returns:
            %   << this (NcsSensor)
            %      A new NcsSensor instance.
            
            this.measModel = measModel;
        end
        
        %% step
        function measurement = step(this, plantState)
            % Take a measurement of the plant state as part of a control cycle in an
            % NCS.
            %
            % Parameters:
            %   >> plantState (Vector)
            %      The true state of the plant.
            %
            % Returns:
            %   << measurement (Column vector, might be empty)
            %      The measurement taken by the sensor.
            %      Empty matrix is returned in case none is taken or to be transmitted (e.g., when the sensor is event-based).
            %
            
            % take a measurement y_k
            measurement = this.measModel.simulate(plantState);
            % perform send-on-delta strategy
            if ~isempty(this.lastSentMeasurement) && norm(this.lastSentMeasurement - measurement) <= this.measurementDelta
                % do not send the measurement
                measurement = [];
            else
                this.lastSentMeasurement = measurement;
            end
        end
    end
    
end

