function structOut = ncs_doHandleQocTarget(ncsHandle, qocTarget)
    % Receive a QoC setpoint from the CoCC congestion control.
    %
    % Parameters:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a
    %      NetworkedControlSystem the received packet is associated with.
    %
    %   >> qocTarget (Nonnegative scalar)
    %      The new QoC setpoint for the NCS. 
    %
    % Returns:
    %   << structOut (Struct, might be empty)
    %      If non-empty, the following field is present:
    %      - samplingInterval (Positive integer), indicating the sampling rate (in pico-seconds) 
    %        used by the controller to reach the given QoC setpoint.

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
        qocTarget(1,1) double {mustBeNonnegative}
    end
    
    ncs = GetNcsByHandle(ncsHandle); % crashes if handle is invalid
    
    % based on the internal relationship between QoC and data rate, change
    % either the employed event-trigger (threshold) or the sampling rate    
    if ncs.controller.isEventBased && isprop(ncs.controller, 'deadband')
        %quick and dirty hack: hard code function obtained from data for
        %pendulum
        func = cfit(fittype('a*exp(-((x-b)/c)^2)'), 0.01054, -0.3651, 0.6821);
    
        newThreshold = func(qocTarget);
        ncs.controller.deadband = max(0, newThreshold);
        
        structOut = struct.empty(); 
    else        
        % adapt by changing the sampling rate and feed back change and feed back change to Omnet
        [structOut.samplingInterval, structOut.deviationFactor] = ncs.handleTargetQoc(qocTarget);        
    end
end

