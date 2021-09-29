function dataPacket = CreateDataPacket(payload, timestamp, srcAddress, dstAddress, isAck, packetId) 
    % Convenience function to create a Matlab DataPacket.
    %
    % Parameters:
    %   >> payload 
    %      The payload of the new DataPacket.
    %
    %   >> timestamp (Positive Integer)
    %      A positive integer indicating the time (time step with respect to the corresponding NCS) 
    %      this packet is created.
    %
    %   >> srcAddr (Positive Integer)
    %      The source address, i.e., the Matlab address/identifier of the
    %      sender.
    %
    %   >> dstAddr (Positive Integer)
    %      The destination address, i.e., the Matlab address/identifier of the
    %      receiver.
    %
    %   >> isAck (Logical, optional)
    %      A logical value indicating whether this packet is an ACK
    %      (acknowledgment).
    %
    %   >> packetId (Positive Integer, optional)
    %      A positive integer indicating the (not necessarily unique) identifier of
    %      the packet.
    %
    % Returns:
    %   << packet (DataPacket)
    %      The created DataPacket instance.   
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
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
   
    switch nargin
        case 5
            dataPacket = DataPacket(payload, timestamp);
            dataPacket.isAck = isAck;
        case 6
            dataPacket = DataPacket(payload, timestamp, packetId);
            dataPacket.isAck = isAck;
        otherwise
            dataPacket = DataPacket(payload, timestamp);
    end
    dataPacket.sourceAddress = srcAddress;
    dataPacket.destinationAddress = dstAddress;
end

