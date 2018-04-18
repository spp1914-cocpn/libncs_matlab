function id = ncs_pktGetId(packet)
    % Returns the identifier of the given packet. 
    % The identifier is not guaranteed to be unique.
    %
    % Parameters:
    %   >> packet (DataPacket)
    %      A DataPacket instance. 
    %
    % Returns:
    %   << id (Positive integer)
    %      A positive integer, packet identifier.

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
    
	% identifier stored in $packet
    assert(Checks.isClass(packet, 'DataPacket'), ...
        'ncs_pktGetId:InvalidPacket', ...
        '** <packet> expected to be a single DataPacket **'); 

    id = packet.id;
end

