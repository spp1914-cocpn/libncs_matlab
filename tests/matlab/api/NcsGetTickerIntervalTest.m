classdef NcsGetTickerIntervalTest < matlab.unittest.TestCase
    % Test cases for the api function ncs_getTickerInterval.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        ncs;
        ncsHandle;
        tickerInterval;
        
        componentMap;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(this)
            this.componentMap.clear();
            % required to destroy the singleton instance
            clear ComponentMap;
        end
    end
    
    methods (TestMethodSetup)
        %% ncs_getTickerInterval
        function init(this)
            this.componentMap = ComponentMap.getInstance();            
            this.tickerInterval = .2; % 0.2s
            this.ncs = NetworkedControlSystem('NCS', this.tickerInterval, NetworkType.TcpLike);
            this.ncsHandle = this.componentMap.addComponent(this.ncs);

            this.addTeardown(@tearDown, this);
        end
    end
    
    methods (Test)
        %% testInvalidHandle
        function testInvalidHandle(this)
            expectedErrId = 'ComponentMap:InvalidComponentType';
            
            invalidHandle = this.componentMap.addComponent(this); % invalid type
            this.verifyError(@() ncs_getTickerInterval(invalidHandle), expectedErrId);
            
            expectedErrId = 'ComponentMap:InvalidIndex';
            
            invalidHandle = this.ncsHandle + 2; % not a valid index
            this.verifyError(@() ncs_getTickerInterval(invalidHandle), expectedErrId);
        end
        
        %% test
        function test(this)
            expectedTickerInterval = this.tickerInterval * 1e12; % in pico-seconds
            actualTickerInterval = ncs_getTickerInterval(this.ncsHandle);
            
            this.verifyEqual(actualTickerInterval, expectedTickerInterval);
        end
    end
end

