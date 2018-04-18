function results = executeApiTests()
    % Function to run all test cases in libncs_matlab/tests/matlab/api.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2018  Florian Rosenthal <florian.rosenthal@kit.edu>
    %
    %                        Institute for Anthropomatics and Robotics
    %                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
    %                        Karlsruhe Institute of Technology (KIT), Germany
    %
    %                        http://isas.uka.de
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

    import matlab.unittest.TestSuite;
    
    tests = [
            TestSuite.fromClass(?NcsDoHandlePacketTest) ...
            TestSuite.fromClass(?NcsDoLoopStepTest) ...
            TestSuite.fromClass(?NcsFinalizeTest) ...
            TestSuite.fromClass(?NcsGetTickerIntervalTest) ...
            TestSuite.fromClass(?NcsPktCreateTest) ...
            TestSuite.fromClass(?NcsPktGetDstAddrTest) ...
            TestSuite.fromClass(?NcsPktGetIdTest) ...
            TestSuite.fromClass(?NcsPktGetPayloadTest) ...
            TestSuite.fromClass(?NcsPktGetSrcAddrTest) ...
            TestSuite.fromClass(?NcsPktGetTimeStampTest) ...
            TestSuite.fromClass(?NcsPktIsAckTest) ...
            TestSuite.fromClass(?NcsSeedRngTest)
        ];

    results = tests.run();

end

