classdef NcsPktGetDstAddrTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_pktGetDstAddr.
    
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
        
    properties (Access = private)
        dataPacket;
        
        payload;
        timestamp;
        id;
        destinationAddress;
        expectedTranslatedDestinationAddress;
    end
    
     methods (TestMethodSetup)
        %% init
        function init(this)
            this.payload = [1 2 3]';
            this.timestamp = 10;
            this.id = 1;
            this.destinationAddress = 6;
            this.expectedTranslatedDestinationAddress = this.destinationAddress - 1;
            
            this.dataPacket = DataPacket(this.payload, this.timestamp, this.id); 
            this.dataPacket.destinationAddress = this.destinationAddress;
        end
     end
    
    methods (Test)
        %% testInvalidDataPacket
        function testInvalidDataPacket(this)
            expectedErrId = 'ncs_pktGetDstAddr:InvalidPacket';
            
            invalidPacket = this; % not a data packet
            this.verifyError(@() ncs_pktGetDstAddr(invalidPacket), expectedErrId);
            
            invalidPacket = [this.dataPacket; this.dataPacket]; % not a single data packet
            this.verifyError(@() ncs_pktGetDstAddr(invalidPacket), expectedErrId);
        end
        
        %% test
        function test(this)
            import matlab.unittest.constraints.IsScalar
            actualTranslatedDestinationAddress = ncs_pktGetDstAddr(this.dataPacket);
            
            this.verifyThat(actualTranslatedDestinationAddress, IsScalar);
            this.verifyGreaterThanOrEqual(actualTranslatedDestinationAddress, 0);
            this.verifyEqual(actualTranslatedDestinationAddress, this.expectedTranslatedDestinationAddress);
        end
    end
end

