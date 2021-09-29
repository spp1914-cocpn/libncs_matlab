function pktsOut = ncs_doHandlePacket(handle, timestamp, pktIn)
    % Receive and buffer a Matlab DataPacket until next loop cycle is triggered (i.e., ncs_doLoopStep is called). 
    %
    % Parameters:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a
    %      NetworkedControlSystem the received packet is associated with.
    %
    %   >> timestamp (Positive Integer)
    %      The current simulation time (in Omnet), in pico-seconds.   
    %
    %   >> pktIn (DataPacket)
    %      The DataPacket to buffer.
    %
    % Returns:
    %   << pktsOut (Cell array, column-vector like, of DataPackets, currently empty)
    %      The DataPackets resulting from the reception of the given packet
    %      sent (e.g., acknowledgments), row-wise arranged in a cell array
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    %      
    %   Markus Jung, Florian Rosenthal, and Martina Zitterbart,
    %   CoCPN-Sim: An Integrated Simulation Environment for Cyber-Physical Systems,
    %   Proceedings of the 2018 IEEE/ACM Third International Conference on Internet-of-Things Design and Implementation (IoTDI), 
    %   Orlando, FL, USA, April 2018.
    
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
        handle {GetNcsByHandle(handle)} % crashes if handle is invalid
        timestamp(1,1) double {mustBePositive, mustBeInteger}
        pktIn(1,1) DataPacket
    end

    DataPacketBuffer.getInstance().addPacket(handle, pktIn);
    % so far, do not return any ACKs or other packets
    pktsOut = cell.empty();
end

