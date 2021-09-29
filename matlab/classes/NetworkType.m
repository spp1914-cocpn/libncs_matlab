classdef NetworkType < uint8
    % Enum to provide constants indicating what type of network is in use.
        
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
    
    enumeration
        TcpLike (1), % ACKs are received instantaneously -> mode history up to k-1 available at time k
        UdpLike (2), % ACKs are not sent at all -> true mode never available
        UdpLikeWithAcks (3) % ACKs are delayed and lossy as well -> subset of the mode history available at time k
    end
    
    properties
    end
    
    methods (Access = public)
        %% previousPlantModeAvailable
        function ret = previousPlantModeAvailable(this)
            ret = (this == NetworkType.TcpLike);
        end
        
        %% sendOutAck
        function ret = sendOutAck(this)
            % we do not send ACKs back to the controller in case of
            % TCP-like or UDP-like communication
            ret = (this == NetworkType.UdpLikeWithAcks);
        end
    end
    
    methods (Access = public, Static)
        %% getMaxIdx
        function maxId = getMaxId()
            maxId = 3;
        end
    end
    
end

