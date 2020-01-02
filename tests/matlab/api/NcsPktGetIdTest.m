classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsPktGetIdTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_pktGetId.
    
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
        dataPacket;
        
        payload;
        timestamp;
        id;
    end
    
     methods (TestMethodSetup)
        %% init
        function init(this)
            this.payload = [1 2 3]';
            this.timestamp = 1;
            this.id = 42;
            
            this.dataPacket = DataPacket(this.payload, this.timestamp, this.id); 
        end
     end
    
    methods (Test)
        %% testInvalidDataPacket
        function testInvalidDataPacket(this)
            expectedErrId = 'ncs_pktGetId:InvalidPacket';
            
            invalidPacket = this; % not a data packet
            this.verifyError(@() ncs_pktGetId(invalidPacket), expectedErrId);
            
            invalidPacket = [this.dataPacket; this.dataPacket]; % not a single data packet
            this.verifyError(@() ncs_pktGetId(invalidPacket), expectedErrId);
        end
        
        %% test
        function test(this)
            actualId = ncs_pktGetId(this.dataPacket);
            
            import matlab.unittest.constraints.IsScalar
            
            this.verifyThat(actualId, IsScalar);
            this.verifyGreaterThan(actualId, 0);
            this.verifyEqual(actualId, this.id);
        end
    end
end

