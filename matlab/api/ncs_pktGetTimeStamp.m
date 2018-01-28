function timestamp = ncs_pktGetTimeStamp(packet)
    % Gets the time stamp of the given Matlab DataPacket, indicating the
    % time step/cycle (with respect to the corresponding NCS/sender) the packet was created. 
    %
    % Parameters:
    %   >> packet (DataPacket)
    %      A DataPacket instance. 
    %
    % Returns:
    %   << timestamp (Positive integer)
    %      A positive integer indicating the NCS cycle this packet is
    %      was created.
    
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
    
    if ~Checks.isClass(packet, 'DataPacket')
        error('ncs_pktGetTimeStamp:InvalidPacket', ...
              '** <packet> expected to be a single DataPacket **'); 
    end
    timestamp = packet.timeStamp;
end

