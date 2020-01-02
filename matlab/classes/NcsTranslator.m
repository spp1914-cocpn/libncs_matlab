classdef NcsTranslator < handle
    % This class implements the control-related portion of the CoCPN-Translator.
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2019  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (SetAccess = immutable, GetAccess = private)
        qocRateCurve = [];%@fittype; % to be extended to sfit, rate = r(QoC_actual, QoC_desired)
        controlErrorQocCurve = []; % function f: qoc=f(error)
        maxPossibleRate;
    end
    
    methods (Access = public)
        %% NcsTranslator
        function this = NcsTranslator(qualityRateCurve, controlErrorQualityCurve, maxPossibleRate)
            % Class constructor.
            %
            % Parameters:
            %   >> qualityRateCurve (cfit or sfit)
            %      Relationship between average control performance, 
            %      expressed in terms of the normalized Quality of Control
            %      (QoC), which is in [0,1], and the average comminication rate (in packets/second) required to reach it. 
            %            
            %   >> controlErrorQualityCurve (cfit or sfit)
            %      Relationship between control error (application-dependent, as recorded by the controller), 
            %      and the normalized Quality of Control
            %      (QoC) measure, which is in [0,1].
            %
            %   >> maxPossibleRate (Positive scalar)
            %      The maximum possible communication rate (in packets per
            %      second) that can be achieved by the associated NCS (or, more, precisely, by the controller).
            % 
            % Returns:
            %   << this (NcsTranslator)
            %      A new NcsTranslator instance.
            
            assert(Checks.isClass(qualityRateCurve, 'fittype'), ...
                'NcsTranslator:InvalidQualityRateCurve', ...
                '** <qualityRateCurve> must denote a fitted curve, i.e., a cfit or sfit instance. **');            
            assert(Checks.isClass(controlErrorQualityCurve, 'fittype'), ...
                'NcsTranslator:InvalidControlErrorQualityCurve', ...
                '** <controlErrorQualityCurve> must denote a fitted curve, i.e., a cfit or sfit instance. **');
            assert(Checks.isPosScalar(maxPossibleRate) && isfinite(maxPossibleRate), ...
                'NcsTranslator:InvalidMaxPossibleRate', ...
                '** <maxPossibleRate> denotes a packet rate und must thus be a positive scalar. **');
            
            this.qocRateCurve = qualityRateCurve;
            this.controlErrorQocCurve = controlErrorQualityCurve;
            this.maxPossibleRate = maxPossibleRate;
        end
        
        %% getDataRateForQoC
        function [rate, rateChange] = getDataRateForQoc(this, actualQoC, targetQoC)
            % Get the data rate required to reach a target control performance (QoC) given an actual QoC
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> actualQoC (Nonnegative scalar)
            %      The current actual QoC of the NCS (or the controller's estimate thereof), given in terms of a
            %      value in [0,1].
            %
            %   >> targetQoC (Nonnegative scalar)
            %      The desired QoC of the NCS, given in terms of a
            %      value in [0,1].
            %
            % Returns:
            %   << rate (Nonnegative scalar)
            %      The data rate (in packets per second) required to
            %      reach the target QoC, based on the underlying communication
            %      characteristics. The maximum value to be returned is
            %      this.maxPossibleRate.
            %
            %   << rateChange (Nonnegative scalar, optional)
            %      The partial derivative of the underlying communication
            %      model with regards to the target QoC, evaluated at the
            %      given value.
            
            % so far, the curve is static
            rate = this.qocRateCurve(targetQoC);
                       
            %normalize rate: nonnegative and <= maxPossibleRate packets per second
            rate = min(max(0, rate), this.maxPossibleRate);
            
            if nargout == 2
                % differentiate with regards to targetQoC (i.e., partial
                % derivative)
                rateChange = differentiate(this.qocRateCurve, targetQoC);
            end
        end
        
        %% getQocForDataRate
        function qoc = getQocForDataRate(this, actualQoc, targetRate)
            % Get the control performance (QoC) that can be achieved, given an actual QoC and a desired data rate,
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> actualQoC (Nonnegative scalar)
            %      The current actual QoC of the NCS (or the controller's estimate thereof), given in terms of a
            %      value in [0,1].
            %
            %   >> targetRate (Nonnegative scalar)
            %      The desired data rate (in packets per second).
            %
            % Returns:
            %   << qoc (Nonnegative scalar)
            %      The QoC that can be achieved given the values of actual QoC and desired data rate,
            %      computed based on the underlying communication characteristics of the NCS.
             
            % we want the QoC value that can be achieved with the given rate
            % when the current actual QoC is known
            
            % so far, the curve is static, independent of actualQoC
            x0 = 0.5;
            % numerically, find the inverse (translation into root finding problem)
            qoc = fzero(@(x) this.qocRateCurve(x) - targetRate, x0);
            % clamp to ensure that return value is nonnegative
            qoc = max(0, qoc);
        end
        
        %% translateControlError
        function qoc = translateControlError(this, controlError)
            % qoc is in unit interval [0,1]
            qoc = min(1, max(this.controlErrorQocCurve(controlError), 0));
        end
    end
    
end

