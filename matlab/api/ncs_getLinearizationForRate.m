function [slope, yIntercept] = ncs_getLinearizationForRate(ncsHandle, actualQoC, desiredQoC)
    % Provide the parameters of a simplified relationship, namely an affine relationship of the form y=m*x+b,
    % between desired Quality of Control and required data rate that is used for congestion
    % control computations.
    % The parameters are obtained by a first order Taylor expansion of the real communication characteristics
    % w.r.t. desired QoC, evaluated at the given values with the actual QoC being a constant.
    % In general, this relationship is thus only an approximation of the real communication behavior of the NCS.
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
    %   << slope (Scalar)
    %      The slope parameter (m) of the assumed linear relationship.
    %
    %   << yIntercept (Scalar)
    %      The yIntercept parameter of the assumed linear relationship.

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
    
    [rate, rateChange] = ncs.evaluateRateQualityCharacteristics(desiredQoC);
    
    yIntercept = rate - rateChange * desiredQoC;
    slope = rateChange; % derivative of rate relationship
end

