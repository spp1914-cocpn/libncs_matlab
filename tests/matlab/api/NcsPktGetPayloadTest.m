classdef NcsPktGetPayloadTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_pktGetPayload.
    
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
        
        expectedByteStream;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.payload = [1 2 3]';
            this.timestamp = 1;
            this.id = 42;
            
            this.dataPacket = DataPacket(this.payload, this.timestamp, this.id);           
            
            % corresponds to the given payload, timestamp and id and isAck = false
            this.expectedByteStream = uint8([ ...
                0     1    73    77     0     0     0     0    14     0     0     0    48     1     0     0     6     0     0     0     8 ...
                0     0     0     1     0     0     0     0     0     0     0     5     0     0     0     8     0     0     0     4     0 ...
                0     0     1     0     0     0     1     0     0     0     0     0     0     0    14     0     0     0    56     0     0 ...
                0     6     0     0     0     8     0     0     0     6     0     0     0     0     0     0     0     5     0     0     0 ...
                8     0     0     0     1     0     0     0     1     0     0     0     1     0     0     0     0     0     0     0     9 ...
                0     0     0     8     0     0     0     0     0     0     0     0     0   240    63    14     0     0     0    48     0 ...
                0     0     6     0     0     0     8     0     0     0     9     2     0     0     0     0     0     0     5     0     0 ...
                0     8     0     0     0     1     0     0     0     1     0     0     0     1     0     0     0     0     0     0     0 ...
                2     0     1     0     0     0     0     0    14     0     0     0    56     0     0     0     6     0     0     0     8 ...
                0     0     0     6     0     0     0     0     0     0     0     5     0     0     0     8     0     0     0     1     0 ...
                0     0     1     0     0     0     1     0     0     0     0     0     0     0     9     0     0     0     8     0     0 ...
                0     0     0     0     0     0     0    69    64    14     0     0     0    72     0     0     0     6     0     0     0 ...
                8     0     0     0     6     0     0     0     0     0     0     0     5     0     0     0     8     0     0     0     3 ...
                0     0     0     1     0     0     0     1     0     0     0     0     0     0     0     9     0     0     0    24     0 ...
                0     0     0     0     0     0     0     0   240    63     0     0     0     0     0     0     0    64     0     0     0 ...
                0     0     0     8    64 ...
                ]);
        end
    end
    
    methods (Test)
        %% testInvalidDataPacket
        function testInvalidDataPacket(this)
            expectedErrId = 'ncs_pktGetPayload:InvalidPacket';
            
            invalidPacket = this; % not a data packet
            this.verifyError(@() ncs_pktGetPayload(invalidPacket), expectedErrId);
            
            invalidPacket = [this.dataPacket; this.dataPacket]; % not a single data packet
            this.verifyError(@() ncs_pktGetPayload(invalidPacket), expectedErrId);
        end
        
        %% test
        function test(this)
            this.assertFalse(this.dataPacket.isAck);
            actualByteStream = ncs_pktGetPayload(this.dataPacket);            
            
            this.assertSize(actualByteStream, size(this.expectedByteStream));
            this.verifyEqual(actualByteStream, this.expectedByteStream);
        end
    end
    
end

