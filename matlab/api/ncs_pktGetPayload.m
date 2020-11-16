function payload = ncs_pktGetPayload(packet)
    % Create the payload for an Omnet RawPacket, which consists of
    % a serialized 4x1 cell array containing timestamp, ack information, id and payload of the given Matlab DataPacket.
    %
    % Parameters:
    %   >> packet (DataPacket)
    %      A DataPacket instance. 
    %
    %
    % Returns:
    %   << payload (uint8 Row vector)
    %      A byte stream (i.e., a uint8 row vector) which
    %      represents the serialized 4x1 cell array consisting of timestamp, ack information and
    %      payload of the given packet.
    
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
    
    arguments
        packet(1,1) DataPacket;
    end 

    % valid data packet
    % get the payload (and the timestamp + ack) and serialize it
    payload = getByteStreamFromArray({packet.timeStamp; packet.isAck; packet.id; packet.payload});
end

