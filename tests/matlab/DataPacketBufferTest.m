classdef DataPacketBufferTest < matlab.unittest.TestCase
    % Test cases for DataPacketBuffer.
    
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
        packetPayload;
        packetTimestamp;
        packetId;
        dataPacket@DataPacket;
        dataPacket2@DataPacket;
        
        key1;
        key2;
        
        packetBufferUnderTest;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(~)
            % required to destroy the singleton instance
            clear DataPacketBuffer;
        end
        
        %% verifyErrorInvalidKey
        function verifyErrorInvalidKey(this, func, expectedErrId, varargin)
            invalidKey = this; % must be a scalar
            this.verifyError(@() func(this.packetBufferUnderTest, invalidKey, varargin{:}), expectedErrId);
            
            invalidKey = -1; % must be positive
            this.verifyError(@() func(this.packetBufferUnderTest, invalidKey, varargin{:}), expectedErrId);
            
            invalidKey = 2.5; % must be an integer
            this.verifyError(@() func(this.packetBufferUnderTest, invalidKey, varargin{:}), expectedErrId);
            
            invalidKey = inf; % must be a finite value
            this.verifyError(@() func(this.packetBufferUnderTest, invalidKey, varargin{:}), expectedErrId);
            
        end
    end
       
    methods (TestMethodSetup)
        %% initProperties
        function init(this)
            this.key1 = 10;
            this.key2 = 42;
            
            this.packetPayload = 1;
            this.packetTimestamp = 1;
            this.packetId = 1;
            this.dataPacket = DataPacket(this.packetPayload, this.packetTimestamp, this.packetId);
            this.dataPacket2 = DataPacket(this.packetPayload, this.packetTimestamp + 1, this.packetId + 1);
            this.packetBufferUnderTest = DataPacketBuffer.getInstance();
            
            this.addTeardown(@() this.tearDown());
        end
    end
    
    methods (Test)
        
        %% testClearInvalidKey
        function testClearInvalidKey(this)
            expectedErrId = 'DataPacketBuffer:Clear:InvalidKey';
                       
            func = @clear;
            this.verifyErrorInvalidKey(func, expectedErrId);
        end
        
        %% testClear
        function testClear(this)
            % first, add packets for both keys
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket2);
            this.packetBufferUnderTest.addPacket(this.key2, this.dataPacket2);
            
            % now call clear for a non-existing key
            key = 11;
            this.packetBufferUnderTest.clear(key);
            
            % key1 and key2 should not be affected
            this.verifyNotEmpty(this.packetBufferUnderTest.getDataPackets(this.key1));
            this.verifyNotEmpty(this.packetBufferUnderTest.getDataPackets(this.key2));
            
            % now call clear for key1
            this.packetBufferUnderTest.clear(this.key1);
            
            this.verifyEmpty(this.packetBufferUnderTest.getDataPackets(this.key1));
            this.verifyNotEmpty(this.packetBufferUnderTest.getDataPackets(this.key2));
        end
        
        %% testClearAll
        function testClearAll(this)
            % first, add packets for both keys
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket2);
            this.packetBufferUnderTest.addPacket(this.key2, this.dataPacket2);
            
            % clear and check if buffer is empty
            this.packetBufferUnderTest.clearAll();
            this.verifyEmpty(this.packetBufferUnderTest.getDataPackets(this.key1));
            this.verifyEmpty(this.packetBufferUnderTest.getDataPackets(this.key2));
        end
        
        %% testGetDataPacketsEmptyPackets
        function testGetDataPacketsEmptyPackets(this)
            key = this; % invalid keys should not result in an error here
            this.verifyEmpty(this.packetBufferUnderTest.getDataPackets(key));
            
            key = this.key2; 
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            % for the given key, no packets should be present
            this.verifyEmpty(this.packetBufferUnderTest.getDataPackets(key));
        end
        
        %% testGetDataPackets
        function testGetDataPackets(this)
            % first, add two packets for key1 and one for key2
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket2);
            this.packetBufferUnderTest.addPacket(this.key2, this.dataPacket2);
            
            actualBufferedPackets = this.packetBufferUnderTest.getDataPackets(this.key1);
            this.verifySize(actualBufferedPackets, [1 2]);
            this.verifyEqual(actualBufferedPackets(1), this.dataPacket);
            this.verifyEqual(actualBufferedPackets(2), this.dataPacket2);
        end
        
        %% testAddPacketInvalidPacket
        function testAddPacketInvalidPacket(this)
            expectedErrId = 'DataPacketBuffer:AddPacket:InvalidDataPacket';
            
            invalidPacket = zeros(4);
            this.verifyError(@() this.packetBufferUnderTest.addPacket(this.key1, invalidPacket), expectedErrId);
        end
        
        %% testAddPacketInvalidKey
        function testAddPacketInvalidKey(this)
            expectedErrId = 'DataPacketBuffer:AddPacket:InvalidKey';
                       
            func = @addPacket;
            this.verifyErrorInvalidKey(func, expectedErrId, this.dataPacket);
        end
        
        %% testAddPacketKeyNotPresent
        function testAddPacketKeyNotPresent(this)
            % initially, there should be no packet buffered for the given
            % key
            this.assertEmpty(this.packetBufferUnderTest.getDataPackets(this.key1));
            
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            
            actualBufferedPackets = this.packetBufferUnderTest.getDataPackets(this.key1);
            this.verifyNotEmpty(actualBufferedPackets);
        end
        
        %% testAddPacketKeyAlreadyPresent
        function testAddPacketKeyAlreadyPresent(this)
            % first, ensure that the buffer is non-empty
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket);
            packets = this.packetBufferUnderTest.getDataPackets(this.key1);
            this.assertNotEmpty(packets);
            
            % now add a packet and verify that the number of buffered
            % elements increased by 1
            this.packetBufferUnderTest.addPacket(this.key1, this.dataPacket2);
            actualBufferedPackets = this.packetBufferUnderTest.getDataPackets(this.key1);
            this.verifyEqual(numel(actualBufferedPackets), numel(packets) + 1);
        end
        
        %% testGetInstance
        function testGetInstance(this)
            newInstance = DataPacketBuffer.getInstance();
                        
            this.verifySameHandle(newInstance, this.packetBufferUnderTest);
            
            clear DataPacketBuffer;
            newInstance = DataPacketBuffer.getInstance();
            this.verifyNotSameHandle(newInstance, this.packetBufferUnderTest);
        end
    end
    
end

