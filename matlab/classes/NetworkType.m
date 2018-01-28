classdef NetworkType
    
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
        TcpLike, % ACKs are received instantaneously -> mode history up to k-1 available at time k
        UdpLike, % ACKs are not sent at all -> true mode never available
        UdpLikeWithAcks % ACKs are delayed and lossy as well -> subset of the mode history available at time k
    end
    
    properties
    end
    
    methods
    end
    
end

