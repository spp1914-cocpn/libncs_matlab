classdef NcsSensorTest < matlab.unittest.TestCase
    % Test cases for NcsSensor.
    
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
        dimMeas;
        dimPlant;
        C;
        V;
        measModel;
        ncsSensorUnderTest;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.dimMeas = 1;
            this.dimPlant = 3;
            this.C = [1 2 3];
            this.V = 0.1^2; % variance of the meas noise
     
            this.measModel = LinearMeasurementModel(this.C);
            this.measModel.setNoise(Gaussian(0, this.V));
            this.ncsSensorUnderTest = NcsSensor(this.measModel);
        end
    end
    
    methods (Test)
        %% testNcsSensor
        function testNcsSensor(this)
            ncsSensor = NcsSensor(this.measModel);
            
            this.verifyFalse(ncsSensor.isEventBased);
        end
        
                
        %% testStep
        function testStep(this)
            import matlab.unittest.constraints.IsScalar;
            plantState = [2 3 4]';
            timestep = 1;
            
            rng(1); % seed
            measurementPacket = this.ncsSensorUnderTest.step(timestep, plantState);
            
            this.verifyNotEmpty(measurementPacket);
            this.verifyClass(measurementPacket, ?DataPacket);
            measurement = measurementPacket.payload;
            % we can only verify that the measurement is a scalar
            this.verifyThat(measurement, IsScalar);
            % check timestep, source (id=3) and destination (id=2) of
            % packet
            this.verifyEqual(measurementPacket.timeStamp, timestep)
            this.verifyEqual(measurementPacket.sourceAddress, 3)
            this.verifyEqual(measurementPacket.destinationAddress, 2)
            
            % take a measurement for the same plant state again
            % and use the same seed to obtain the same measurement
            rng(1);
            measurementPacket2 = this.ncsSensorUnderTest.step(timestep, plantState);
            this.verifyNotEmpty(measurementPacket2);
            this.verifyEqual(measurementPacket.timeStamp, timestep)
            this.verifyEqual(measurementPacket.sourceAddress, 3)
            this.verifyEqual(measurementPacket.destinationAddress, 2)
        end
    end
end

