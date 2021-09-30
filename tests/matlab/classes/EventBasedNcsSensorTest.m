classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        EventBasedNcsSensorTest < matlab.unittest.TestCase
    % Test cases for EventBasedNcsSensor.
    
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
    
   properties (Access = private)
        dimMeas;
        dimPlant;
        C;
        V;
        measModel;
        ncsSensorUnderTest;
        defaultMeasDelta;
        
        noiseSamples;
    end
    
    methods (Access = private)
        %% setMeasDelta
        function setMeasDelta(this, measDelta)
            this.ncsSensorUnderTest.measurementDelta = measDelta;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.dimMeas = 1;
            this.dimPlant = 3;
            this.C = [1 2 3];
            this.V = 0.1^2; % variance of the meas noise
            this.defaultMeasDelta = 2;
     
            rng(10); % seed, for repeatability
            maxSensorSteps = 1000;
            measNoise = Gaussian(0, this.V);
            this.noiseSamples = measNoise.drawRndSamples(maxSensorSteps);
            
            rng(10); % seed, for repeatability
            this.measModel = LinearMeasurementModel(this.C);
            this.measModel.setNoise(measNoise);
            
            this.ncsSensorUnderTest = EventBasedNcsSensor(this.measModel, maxSensorSteps, this.defaultMeasDelta);
        end
    end
    
    methods (Test)
        %% testEventBasedNcsSensor
        function testEventBasedNcsSensor(this)
            ncsSensor = EventBasedNcsSensor(this.measModel, 100);
            % the default setting of the class
            this.verifyEqual(ncsSensor.measurementDelta, 10);
            this.verifyTrue(ncsSensor.isEventBased);
        end
        
        %% testSetMeasurementDelta
        function testSetMeasurementDelta(this)
            if verLessThan('matlab', '9.8')
                % Matlab R2018 or R2019
                expectedErrId = 'MATLAB:UnableToConvert';
            else
                expectedErrId = 'MATLAB:validation:UnableToConvert';
            end
            
            invalidMeasDelta = this; % not a scalar
            this.verifyError(@() this.setMeasDelta(invalidMeasDelta), expectedErrId);
            
            invalidMeasDelta = -eps; % not nonnegative
            this.verifyError(@() this.setMeasDelta(invalidMeasDelta), 'MATLAB:validators:mustBeNonnegative');
            
            % now a succesfult try
            newMeasDelta = 1000;
            this.setMeasDelta(newMeasDelta)
            this.verifyEqual(this.ncsSensorUnderTest.measurementDelta, newMeasDelta);
        end
        
        %% testStep
        function testStep(this)
            import matlab.unittest.constraints.IsScalar;
            plantState = [2 3 4]';
            detMeasurement = this.C * plantState;
            timestep = 1;
            this.assertEqual(this.ncsSensorUnderTest.measurementDelta, this.defaultMeasDelta);
            
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
            
            % take a measurement for the same plant state again
            % and use the same seed to obtain the same measurement
            timestep = timestep + 1;
            measurementPacket2 = this.ncsSensorUnderTest.step(timestep, plantState);
            this.verifyEmpty(measurementPacket2);
        end
    end    
end

