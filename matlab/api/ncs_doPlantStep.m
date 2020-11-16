function plantStateAdmissible = ncs_doPlantStep(handle, timestamp)
    % Perform an invocation of the plant dynamics in Matlab. 
    %
    % Parameters:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem.
    %
    %   >> timestamp (Positive Integer)
    %      The current simulation time (in Omnet), in pico-seconds.
    %
    % Returns:
    %   << plantStateAdmissible (Logical Scalar, i.e. a Flag)
    %      A flag to indicate whether the current true plant state is admissible (e.g., does
    %      not violate any constraints)     
    
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
        handle
        timestamp(1,1) double {mustBePositive, mustBeInteger} % ensure that timestamp is internally treated as double
    end
    ncs = GetNcsByHandle(handle); % crashes if handle is invalid

    ncs.plantStep(timestamp);
    plantStateAdmissible = ncs.isPlantStateAdmissible();
end
