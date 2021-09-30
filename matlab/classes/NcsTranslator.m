classdef NcsTranslator < handle
    % This class implements the control-related portion of the CoCPN-Translator.
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    %
    %   Markus Jung, Florian Rosenthal, and Martina Zitterbart,
    %   CoCPN-Sim: An Integrated Simulation Environment for Cyber-Physical Systems,
    %   Proceedings of the 2018 IEEE/ACM Third International Conference on Internet-of-Things Design and Implementation (IoTDI), 
    %   Orlando, FL, USA, April 2018.
    %
    %   Florian Rosenthal and Uwe D. Hanebeck,
    %   A Control Approach for Cooperative Sharing of Network Resources in Cyber-Physical Systems,
    %   Proceedings of the 2019 IEEE International Conference on Industrial Cyber-Physical Systems (ICPS 2019), 
    %   Taipei, Taiwan, May 2019.
    
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
    
    properties (SetAccess = immutable, GetAccess = private)
        qocRateCurve (1,1) fittype; % to be extended to sfit, rate = r(QoC_actual, QoC_desired)
        controlErrorQocCurve (1,1) fittype; % function f: qoc=f(error)
        maxPossibleRate (1,1) double {mustBePositive, mustBeFinite} = 100;
        
        minAllowedRate (1,1) double {mustBePositive, mustBeFinite} = 0.1;        
    end
  
    properties (Access = public)    
        qocTarget = 1;
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
           
            this.qocRateCurve = qualityRateCurve;
            this.controlErrorQocCurve = controlErrorQualityCurve;
            this.maxPossibleRate = maxPossibleRate;
        end
        
        %% getDataRateForQoC
        function [rate, rateChange] = getDataRateForQoc(this, targetQoc)
            % Get the data rate required to reach a target control performance (QoC)
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> targetQoc (Nonnegative matrix or vector)
            %      The desired QoC of the NCS, given in terms of a
            %      value in [0,1].
            %      This function supports an arbitrary number of values,
            %      passed as vectors or matrices.
            %
            % Returns:
            %   << rate (Nonnegative vector or matrix)
            %      The data rate (in packets per second) required to
            %      reach the respective target QoC, based on the underlying communication
            %      characteristics. The maximum value to be returned is
            %      this.maxPossibleRate.
            %
            %   << rateChange (Nonnegative vector or matrix, optional)
            %      The partial derivative of the underlying communication
            %      model with regards to the target QoC, evaluated at the
            %      given values.
            
            arguments
                this
                targetQoc double {mustBeGreaterThanOrEqual(targetQoc, 0), mustBeLessThanOrEqual(targetQoc, 1)}
            end
            
            [n, m]=size(targetQoc);
                       
            %normalize rate: nonnegative and <= maxPossibleRate packets per second
            rate = min(max(this.minAllowedRate, this.qocRateCurve(targetQoc(:))), this.maxPossibleRate);
            rate = reshape(rate, n, m);
            if nargout == 2
                % differentiate with regards to targetQoC (i.e., partial
                % derivative)
                rateChange = differentiate(this.qocRateCurve, targetQoc);
                rateChange = reshape(rateChange, n, m);
            end
        end
        
        %% getQocForDataRate
        function qoc = getQocForDataRate(this, targetRate)
            % Get the control performance (QoC) that can be achieved, given a desired data rate,
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:           
            %   >> targetRate (Nonnegative vector or matrix)
            %      The desired data rate (in packets per second).
            %      This function supports an arbitrary number of values,
            %      passed as vectors or matrices.
            %
            % Returns:
            %   << qoc (Nonnegative vector or matrix)
            %      The QoC in [0,1] that can be achieved given a desired data rate,
            %      computed based on the underlying communication characteristics of the NCS.
           
            arguments
                this
                targetRate double {mustBeNonnegative}
            end
                        
            % we want the QoC value that can be achieved with the given rate
            qoc = zeros(size(targetRate));
            for j=1:numel(targetRate)
                % so far, the curve is static, independent of actualQoC
                % check if we have to clamp
                if this.qocRateCurve(1) < targetRate(j)
                    qoc(j) = 1;
                elseif this.qocRateCurve(0) <= targetRate(j)
                    % numerically, find the inverse (translation into root finding problem)
                    % there must be a zero (root) of qocRateCurve(x)-y within
                    % [0,1]
                    qoc(j) = fzero(@(x) this.qocRateCurve(x) - targetRate(j), [0 1]);
                end
            end
        end
        
        %% translateControlError
        function qoc = translateControlError(this, controlErrors)
            % Get the control performance (QoC) that corresponds to a given control error
            % according to the underlying communication characteristics (i.e., the model relating data rate and control performance).
            %
            % Parameters:
            %   >> controlErrors (Vector or matrix)
            %      Control errors that shall be translated into QoC.
            %      This function supports an arbitrary number of values,
            %      passed as vectors or matrices.
            %
            % Returns:
            %   << qoc (Nonnegative vector or matrix)
            %      The QoC values in [0,1] corresponding to the given control errors.

            arguments
                this
                controlErrors double {mustBeReal}
            end
            
            % allow +/-inf and +/-nan
            controlErrors(~isfinite(controlErrors)) = inf;
            [n, m]=size(controlErrors);
            % qoc is in unit interval [0,1]
            qoc = min(1, max(this.controlErrorQocCurve(controlErrors(:)), 0));
            qoc = reshape(qoc, n, m);
        end
    end
    
end

