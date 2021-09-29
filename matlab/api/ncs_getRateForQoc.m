function dataRate = ncs_getRateForQoc(ncsHandle, actualQoC, desiredQoC)
    % Estimate the data rate required to reach a target control performance (QoC) given an actual QoC. 
    %
    % Parameters:
    %   >> ncsHandle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem (NCS).
    %
    %   >> actualQoC (Nonnegative scalar)
    %      The current actual QoC of the NCS associated with the given
    %      handle.
    %
    %   >> desiredQoC (Nonnegative scalar)
    %      The desired QoC of the NCS associated with the given
    %      handle.
    %
    % Returns:
    %   << dataRate (Nonnegative scalar)
    %      The estimated data rate (in packets per second) required to
    %      reach the desired QoC, based on the underlying communication
    %      behavior of the NCS. The maximum value to be returned is
    %      1/samplingRate.
    
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
        desiredQoC(1,1) double {mustBeNonnegative}
    end
    ncs = GetNcsByHandle(ncsHandle);  % crashes if handle is invalid
    
    % data rate is to be returned in packets per second
    dataRate = ncs.evaluateRateQualityCharacteristics(desiredQoC);
end

