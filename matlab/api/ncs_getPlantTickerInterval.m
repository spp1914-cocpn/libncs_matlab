function plantInterval = ncs_getPlantTickerInterval(handle)
    % Gets the ticker interval (sampling interval) (in pico-seconds) of the plant a networked control
    % system, i.e., the period between two invocations of the plant
    % dynamics.
    %
    % Parameters:
    %   >> handle (Key into ComponentMap)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem instance.
    %
    % Returns:
    %   << plantInterval (Nonnegative integer)
    %      The ticker interval, expressed in pico-seconds. 
    
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
    
    ncs = GetNcsByHandle(handle);

    % translate sampling interval (given in seconds) into pico-seconds
    plantInterval = ConvertToPicoseconds(ncs.plantSamplingInterval);
end

