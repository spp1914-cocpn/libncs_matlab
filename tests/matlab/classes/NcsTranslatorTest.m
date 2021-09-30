classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        NcsTranslatorTest < matlab.unittest.TestCase
    % Test cases for NcsTranslator.
    
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
            if verLessThan('matlab', '9.8')
                % Matlab R2018 or R2019
                expectedErrId = 'MATLAB:UnableToConvert';
            else
                expectedErrId = 'MATLAB:validation:UnableToConvert';
            end
            
            invalidQocCurve = this; % wrong type
            this.verifyError(@() NcsTranslator(invalidQocCurve, this.controlErrorQocCurve, this.maxDataRate), expectedErrId);           
        end
        
        %% testNcsTranslatorInvalidControlErrorQualityCurve
        function testNcsTranslatorInvalidControlErrorQualityCurve(this)
            if verLessThan('matlab', '9.8')
                % Matlab R2018 or R2019
                expectedErrId = 'MATLAB:UnableToConvert';
            else
                expectedErrId = 'MATLAB:validation:UnableToConvert';
            end
            
            invalidControlErrorQocCurve = this; % wrong type
            this.verifyError(@() NcsTranslator(this.qocRateCurve, invalidControlErrorQocCurve, this.maxDataRate), expectedErrId);
        end
        
        %% testNcsTranslatorInvalidMaxPossibleRate
        function testNcsTranslatorInvalidMaxPossibleRate(this)            
            if verLessThan('matlab', '9.8')
                % Matlab R2018 or R2019
                expectedErrIds = {'MATLAB:UnableToConvert', 'MATLAB:type:InvalidInputSize'};
            else
                expectedErrIds = {'MATLAB:validation:UnableToConvert', 'MATLAB:validation:IncompatibleSize'};
            end
            
            
            invalidRate = this; % wrong type
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrIds{1});
            
            invalidRate = [1 2]; % not a scalar
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), expectedErrIds{2});
            
            invalidRate = 0; % scalar, but not positive
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), 'MATLAB:validators:mustBePositive');
            
            invalidRate = inf; % scalar, but not finite
            this.verifyError(@() NcsTranslator(this.qocRateCurve, this.controlErrorQocCurve, invalidRate), 'MATLAB:validators:mustBeFinite');            
        end
%%
%%
        %% testGetDataRateForQoc
        function testGetDataRateForQoc(this)
            targetQoc = 0.75;
            expectedDataRate = targetQoc ^ 2 + 2 * targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
            
            
            targetQoc = 1; % results in a data rate higher than max rate, so clamped values is expected to be returned
            expectedDataRate = this.maxDataRate;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
        
        %% testGetDataRateForQocMatrix
        function testGetDataRateForQocMatrix(this)
            targetQoc = [0.75 0.5];
            expectedDataRate = targetQoc .^ 2 + 2 .* targetQoc + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
            
            
            targetQoc = [1 0.5; 0.75 0.5; 0.25 0.8]; % results in a data rate higher than max rate, so clamped values is expected to be returned
            expectedDataRate = zeros(3,2);
            expectedDataRate(1) = this.maxDataRate;
            expectedDataRate(2:end) = targetQoc(2:end) .^ 2 + 2 .* targetQoc(2:end) + 3;
            expectedRateChange = 2 * targetQoc + 2; % derivate of qocRateCurve w.r.t. targetQoc
            
            [actualDataRate, actualRateChange] = this.translatorUnderTest.getDataRateForQoc(targetQoc);
            this.verifyEqual(actualDataRate, expectedDataRate, 'AbsTol', 1e-10);
            this.verifyEqual(actualRateChange, expectedRateChange, 'AbsTol', 1e-10);
        end
%%
%%        
        %% testGetQocForDataRate
        function testGetQocForDataRate(this)
            targetRate = this.maxDataRate;
            expectedQoc = sqrt(targetRate - 2) - 1; % evaluate the inverse function, which is y = sqrt(x-2) -1
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            targetRate = 2.1;
            expectedQoc = 0; % expected qoc value should never be negative, so check if clamped correctly
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc);
            
            targetRate = [2.1 5 6]';
            expectedQoc = [0, sqrt(targetRate(2) - 2) - 1, 1]';
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
        end
        
        %% testGetQocForDataRateMatrix
        function testGetQocForDataRateMatrix(this)
            targetRate = [this.maxDataRate this.maxDataRate; this.maxDataRate this.maxDataRate];
            expectedQoc = sqrt(targetRate - 2) - 1; % evaluate the inverse function, which is y = sqrt(x-2) -1
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            targetRate = [2.1 2.1; 2.1 2.1; 2.1 2.1];
            expectedQoc = zeros(size(targetRate)); % expected qoc value should never be negative, so check if clamped correctly
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc);
             
            targetRate = [2.1 5 6; 2.1 5 6];
            expectedQoc = [0, sqrt(targetRate(1, 2) - 2) - 1, 1;0, sqrt(targetRate(2, 2) - 2) - 1, 1];
            
            actualQoc = this.translatorUnderTest.getQocForDataRate(targetRate);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
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
            
            % now check if nan and inf are accepted, should return 0
            controlError = nan;
            expectedQoc = 0;
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = -nan;
            expectedQoc = 0;
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = inf;
            expectedQoc = 0;
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = -inf;
            expectedQoc = 0;
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
        end
        
        %% testTranslateControlErrorVector
        function testTranslateControlErrorVector(this)
            controlError = [2.5 2 2.5];
            expectedQoc = -0.5 .* controlError + 2; % evaluate function            
            
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            % check if clamped correctly
            controlError = [10; 42; 10];
            expectedQoc = [0 0 0]'; % lower bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = [0 0 0 0 0];
            expectedQoc = [1 1 1 1 1]; % upper bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            % add a nan value
            controlError = [2.5 2 nan 2.5];
            expectedQoc = zeros(size(controlError));
            expectedQoc([1 2 4]) = -0.5 .* controlError([1 2 4]) + 2; % evaluate function 
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = [2.5 -nan 2 2.5];
            expectedQoc = zeros(size(controlError));
            expectedQoc([1 3 4]) = -0.5 .* controlError([1 3 4]) + 2; % evaluate function 
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
        end
        
        %% testTranslateControlErrorMatrix
        function testTranslateControlErrorMatrix(this)
            controlError = [2.5 2; 2.5 2.25];
            expectedQoc = -0.5 .* controlError + 2; % evaluate function            
            
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc, 'AbsTol', 1e-10);
            
            % check if clamped correctly
            controlError = [10 42; 10 10; 11 11];
            expectedQoc = [0 0; 0 0; 0 0]; % lower bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = [0 0; 0 0];
            expectedQoc = [1 1; 1 1]; % upper bound
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            % add a nan value
            controlError = [2.5 2; nan 2.5];
            expectedQoc = zeros(size(controlError));
            expectedQoc([1 3 4]) = -0.5 .* controlError([1 3 4]) + 2; % evaluate function 
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
            
            controlError = [2.5 -nan; 2 2.5];
            expectedQoc = zeros(size(controlError));
            expectedQoc([1 2 4]) = -0.5 .* controlError([1 2 4]) + 2; % evaluate function 
            actualQoc = this.translatorUnderTest.translateControlError(controlError);
            this.verifyEqual(actualQoc, expectedQoc);
        end
    end
end

