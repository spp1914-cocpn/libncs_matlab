classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsSensorTest < matlab.unittest.TestCase
    % Test cases for NcsSensor.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C)  2018-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        noiseSamples;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.dimMeas = 1;
            this.dimPlant = 3;
            this.C = [1 2 3];
            this.V = 0.1^2; % variance of the meas noise
     
            rng(1); % seed, for repeatability
            maxSensorSteps = 1000;
            measNoise = Gaussian(0, this.V);
            this.noiseSamples = measNoise.drawRndSamples(maxSensorSteps);
            
            rng(1); % seed, for repeatability
            this.measModel = LinearMeasurementModel(this.C);
            this.measModel.setNoise(measNoise);
            this.ncsSensorUnderTest = NcsSensor(this.measModel, maxSensorSteps);
        end
    end
    
    methods (Test)
        %% testNcsSensor
        function testNcsSensor(this)
            ncsSensor = NcsSensor(this.measModel, 100);
            
            this.verifyFalse(ncsSensor.isEventBased);
        end
        
                
        %% testStep
        function testStep(this)
            import matlab.unittest.constraints.IsScalar;
            plantState = [2 3 4]';
            detMeasurement = this.C * plantState;
            timestep = 1;
         
            measurementPacket = this.ncsSensorUnderTest.step(timestep, plantState);
            
            this.verifyNotEmpty(measurementPacket);
            this.verifyClass(measurementPacket, ?DataPacket);
            measurement = measurementPacket.payload;
            % verify measurement
            this.verifyThat(measurement, IsScalar);
            this.verifyEqual(measurement,  detMeasurement + this.noiseSamples(:, timestep));
            % check timestep, source (id=3) and destination (id=2) of
            % packet
            this.verifyEqual(measurementPacket.timeStamp, timestep)
            this.verifyEqual(measurementPacket.sourceAddress, 3)
            this.verifyEqual(measurementPacket.destinationAddress, 2)
            
            % take a measurement for a different time step but the same state    
            timestep = 999;
            measurementPacket2 = this.ncsSensorUnderTest.step(timestep, plantState);
            this.verifyNotEmpty(measurementPacket2);
            this.verifyEqual(measurementPacket2.payload,  detMeasurement + this.noiseSamples(:, timestep));
            this.verifyEqual(measurementPacket2.timeStamp, timestep)
            this.verifyEqual(measurementPacket2.sourceAddress, 3)
            this.verifyEqual(measurementPacket2.destinationAddress, 2)
        end
    end
end

