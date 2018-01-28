function ncs = GetNcsByHandle(ncsHandle)
    % Convenience function to obtain the NetworkedControlSystem instance asscociated with the given handle.
    % The function crashes if the handle is not related to an NCS.
    %
    % Parameters:
    %   >> ncsHandle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem.
    %
    % Returns:
    %   << ncs (NetworkedControlSystem)
    %      The corresponding NCS.   
    
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
    
    ncs = ComponentMap.getInstance().getComponent(ncsHandle, 'NetworkedControlSystem');
end

