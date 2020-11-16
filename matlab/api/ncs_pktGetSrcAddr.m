function srcAddr = ncs_pktGetSrcAddr(packet)
    % Translate the source address (i.e., the index of a NCS component) of the given DataPacket (Matlab) 
    % into a valid address of a raw network packet (Omnet).
    %
    % Parameters:
    %   >> packet (DataPacket)
    %      A DataPacket instance. 
    %
    % Returns:
    %   << srcAddr (Nonnegative integer)
    %      A nonnegative integer, the translated source address.
    
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

    % addresses start at 1 in Matlab!
    srcAddr = packet.sourceAddress - 1;
end

