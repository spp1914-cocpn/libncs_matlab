classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsSetSingleThreadedTest < matlab.unittest.TestCase
   % Test cases for the api function ncs_setSingleThreaded.
    
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
    
    properties (Access = private)
        numThreads;
    end
        
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.numThreads = maxNumCompThreads();
            
            % reset
            this.addTeardown(@() maxNumCompThreads(this.numThreads));
        end       
    end
    
    methods (Test)
        %% test
        function test(this)            
            this.verifyEqual(ncs_setSingleThreaded(), this.numThreads);
            
            % check the side effect
            this.verifyEqual(maxNumCompThreads(), 1);
            % second call should have no effect
            this.verifyEqual(ncs_setSingleThreaded(), 1);
        end
    end
end

