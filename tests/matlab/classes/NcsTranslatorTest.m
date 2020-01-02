classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsTranslatorTest < matlab.unittest.TestCase
    % Test cases for NcsTranslator.
    
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
    
    properties (Access = private)
        qocRateCurve;
        controlErrorQocCurve;
        maxDataRate;
        
        translatorUnderTest;
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.qocRateCurve = cfit(fittype('a*x^2+b*x+c'), 1, 2, 3);
            this.controlErrorQocCurve = cfit(fittype('-a*x+2'), 0.5); % for simplicity, use linear function
            this.maxDataRate = 5.5;
            
            this.translatorUnderTest = NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, this.maxDataRate);
        end
    end
    
    methods (Test)
        %% testNcsTranslatorInvalidQualityRateCurve
        function testNcsTranslatorInvalidQualityRateCurve(this)
            expectedErrId = 'NcsTranslator:InvalidQualityRateCurve';
            
            invalidQocCurve = this; % wrong type
            this.verifyError(@() NcsTranslator(invalidQocCurve, this.controlErrorQocCurve, this.maxDataRate), expectedErrId);           
        end
        
        %% testNcsTranslatorInvalidControlErrorQualityCurve
        function testNcsTranslatorInvalidControlErrorQualityCurve(this)
            expectedErrId = 'NcsTranslator:InvalidControlErrorQualityCurve';
            
            invalidControlErrorQocCurve = this; % wrong type
            this.verifyError(@() NcsTranslator(this.qocRateCurve, invalidControlErrorQocCurve, this.maxDataRate), expectedErrId);
        end
        
        %% testNcsTranslatorInvalidMaxPossibleRate
        function testNcsTranslatorInvalidMaxPossibleRate(this)
            expectedErrId = 'NcsTranslator:InvalidMaxPossibleRate';
            
            invalidRate = this; % wrong type
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrId);
            
            invalidRate = [1 2]; % not a scalar
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrId);
            
            invalidRate = 0; % scalar, but not positive
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrId);
            
            invalidRate = inf; % scalar, but not finite
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrId);            
        end
%%
%%
        %% testGetDataRateForQoc
        function testGetDataRateForQoc(this)
            actualQoc = 0.5;
            targetQoc = 0.75;
            expectedDataRate = targetQoc ^ 2 + 2 * targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(actualQoc, targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
            
            
            targetQoc = 1; % results in a data rate higher than max rate, so clamped values is expected to be returned
            expectedDataRate = this.maxDataRate;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(actualQoc, targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
%%
%%
        %% testGetQocForDataRate
        function testGetQocForDataRate(this)
            actualQoc = 0.5;
            targetRate = this.maxDataRate;
            expectedQoc = sqrt(targetRate - 2) - 1; % evaluate the inverse function, which is y = sqrt(x-2) -1
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(actualQoc, targetRate);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            targetRate = 2.1;
            expectedQoc = 0; % expected qoc value should never be negative, so check if clamped correctly
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(actualQoc, targetRate);
            this.verifyEqual(actualQoc, expectedQoc);
        end
%%
%%
        %% testTranslateControlError
        function testTranslateControlError(this)
            controlError = 2.5;
            expectedQoc = -0.5 * controlError +2; % evaluate function
            
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            % check if clamped correctly
            controlError = 10;
            expectedQoc = 0; % lower bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = 0;
            expectedQoc = 1; % upper bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
        end
    end
end

