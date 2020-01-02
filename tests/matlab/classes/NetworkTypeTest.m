classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NetworkTypeTest < matlab.unittest.TestCase
    % Test cases for NetworkType.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2019  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        tcpLike;
        udpLike;
        udpLikeWithAcks;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.tcpLike = NetworkType.TcpLike;
            this.udpLike = NetworkType.UdpLike;
            this.udpLikeWithAcks = NetworkType.UdpLikeWithAcks;
        end
    end
    
    methods (Test)
        %% testPreviousPlantModeAvailable
        function testPreviousPlantModeAvailable(this)            
            this.verifyTrue(this.tcpLike.previousPlantModeAvailable());           
            this.verifyFalse(this.udpLike.previousPlantModeAvailable());
            this.verifyFalse(this.udpLikeWithAcks.previousPlantModeAvailable());
        end
        
        %% testSendOutAck
        function testSendOutAck(this)            
            this.verifyFalse(this.tcpLike.sendOutAck());
            this.verifyFalse(this.udpLike.sendOutAck());
            this.verifyTrue(this.udpLikeWithAcks.sendOutAck());
        end
        
        %% testGetMaxId
        function testGetMaxId(this)
            this.verifyEqual(NetworkType.getMaxId(), 3);
        end
    end
    
end

