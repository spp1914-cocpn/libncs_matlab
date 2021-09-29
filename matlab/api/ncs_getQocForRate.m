function achievedQoC = ncs_getQocForRate(ncsHandle, actualQoC, desiredRate)
    % Estimate the QoC that can be reached given an actual QoC and a
    % desired data rate.
    %
    % Parameters:
    %   >> ncsHandle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem (NCS).
    %
    %   >> actualQoC (Nonnegative scalar)
    %      The current actual QoC of the NCS associated with the given
    %      handle.
    %
    %   >> desiredRate (Nonnegative scalar)
    %      The desired data rate (in packets per second).
    %
    % Returns:
    %   << achievedQoC (Nonnegative scalar)
    %      The estimated QoC that can be achieved given the values of actual QoC and desired data rate,
    %      computed based on the underlying communication behavior of the NCS.
    
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
        ncsHandle
        actualQoC(1,1) double {mustBeNonnegative}
        desiredRate(1,1) double {mustBeNonnegative}
    end
    ncs = GetNcsByHandle(ncsHandle);  % crashes if handle is invalid
    
    achievedQoC = ncs.evaluateQualityRateCharacteristics(desiredRate);    
end

