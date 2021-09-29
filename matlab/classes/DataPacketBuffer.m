classdef DataPacketBuffer < handle
    % Singleton implementation of a buffer for data packets received from Omnet via
    % calls to the API function ncs_doHandlePacket.
    % 
    % The received packets are buffered NCS-wise.
    % The individual NCS are identified by their respective handles.
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
    
    properties (Access = private)
        % stores incoming DataPackets per NCS instance
        buffer;
    end
    
    methods (Access = private)
        %% DataPacketBuffer
        function this = DataPacketBuffer()
            % Class constructor.
            %
            % Returns:
            %   << this (DataPacketBuffer)
            %      A new DataPacketBuffer instance.
            this.buffer = containers.Map('KeyType', 'uint32', 'ValueType', 'any');
        end
    end
    
    methods (Access = public)
        %% clear
        function clear(this, ncsHandle)
            % Clear (i.e., remove) all buffered DataPackets sent to the NCS specified by the given handle.
            %
            % Parameters:
            %   >> handle (Key into ComponentMap)
            %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem instance.
            %
            arguments
                this
                ncsHandle(1,1) {mustBeNumeric, mustBePositive, mustBeInteger}
            end
            
            this.buffer(ncsHandle) = [];
        end
        
        %% clearAll
        function clearAll(this)
            % Clear the buffer, i.e. discard all buffered DataPackets.
            
            this.buffer.remove(this.buffer.keys());
        end
        
        %% getDataPackets
        function packets = getDataPackets(this, ncsHandle)
            % Get all buffered DataPackets sent to the NCS specified by the given handle.
            %
            % Parameters:
            %   >> handle (Key into ComponentMap)
            %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem instance.
            %
            % Returns:
            %   >> packets (Row vector of DataPackets or empty matrix)
            %      The DataPackets buffered for the specified NCS.
            %      In case non are buffered, an empty matrix is returned.
            %
            if isKey(this.buffer, ncsHandle)
                packets = this.buffer(ncsHandle);
            else
                packets = [];
            end
        end
        
        %% addPacket
        function addPacket(this, ncsHandle, packet)
            % Buffer a DataPacket sent to the NCS specified by the given handle.
            %
            % Parameters:
            %   >> handle (Key into ComponentMap)
            %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem instance.
            %
            %   >> packet (DataPacket)
            %      The DataPacket sent to the NCS with the specified
            %      handle.
            %
            arguments
                this
                ncsHandle(1,1) {mustBeNumeric, mustBePositive, mustBeInteger}
                packet(1,1) DataPacket
            end
     
            if ~isKey(this.buffer, ncsHandle)
                this.buffer(ncsHandle) = packet;
            else
               this.buffer(ncsHandle) = [this.buffer(ncsHandle), packet];
            end
        end
    end
    
    methods (Static, Access = public)
        %% getInstance
        function instance = getInstance()
            % Get the singleton instance of this class which is
            %     instantiated upon the call of this method.
            %
            % Returns:
            %   >> instance (DataPacketBuffer)
            %     The singleton instance of this class.
            %
            persistent soleInstance
            if isempty(soleInstance) || ~isvalid(soleInstance)
                soleInstance = DataPacketBuffer();
            end
            instance = soleInstance;
        end
    end
    
end

