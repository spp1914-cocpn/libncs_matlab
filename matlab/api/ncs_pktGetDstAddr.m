function dstAddr = ncs_pktGetDstAddr(packet)
    % Translate the destination address (i.e., the index of a NCS component) of the given DataPacket (Matlab) 
    % into a valid address of a raw network packet (Omnet).
    %
    % Parameters:
    %   >> packet (DataPacket)
    %      A DataPacket instance. 
    %
    % Returns:
    %   << dstAddr (Nonnegative integer)
    %      A nonnegative integer, the translated destination address.

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
    
	% return destination address stored in $packet
    if ~Checks.isClass(packet, 'DataPacket')
        error('ncs_pktGetDstAddr:InvalidPacket', ...
              '** <packet> expected to be a single DataPacket **'); 
    end
    % addresses start at 1 in Matlab!
    dstAddr = packet.destinationAddress - 1;
end

