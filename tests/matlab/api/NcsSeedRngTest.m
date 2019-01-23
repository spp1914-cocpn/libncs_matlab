classdef NcsSeedRngTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_seedRng.
    
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
        seed;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.seed = 42;
        end
    end
    
    methods (Test)
        %% testInvalidSeed
        function testInvalidSeed(this)
            expectedErrId = 'ncs_seedRng:InvalidSeed';
            
            invalidSeed = [-1 -1]; % not a scalar            
            this.verifyError(@() ncs_seedRng(invalidSeed), expectedErrId);
            
            invalidSeed = -42; % not nonnegative           
            this.verifyError(@() ncs_seedRng(invalidSeed), expectedErrId);
            
            invalidSeed = -42; % not an integer            
            this.verifyError(@() ncs_seedRng(invalidSeed), expectedErrId);
        end
        
        %% test
        function test(this)
            % retrieve the settings
            prev = rng;
            % call the api function
            ncs_seedRng(this.seed);
           
            after = rng;
            this.verifyTrue(after.Seed == this.seed);
            % type should not be changed
            this.verifyEqual(prev.Type, after.Type);
        end
    end
    
end

