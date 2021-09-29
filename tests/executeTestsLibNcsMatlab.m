function results = executeTestsLibNcsMatlab(runInParallel)
    % Function to run all test cases in libncs_matlab/tests/matlab.
    %
    % Parameters:
    %   >> runInParallel (Flag, (i.e., a logical scalar), Optional)
    %      A flag to indicate whether the some of the test cases shall be ran in
    %      parallel. If left out, the default value <true> is used.
    
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

    arguments
        runInParallel(1,1) logical = true;
    end
    
    import matlab.unittest.TestSuite;
    import matlab.unittest.TestRunner;
    
    tests = [
            TestSuite.fromClass(?ComponentMapTest) ...
            TestSuite.fromClass(?DataPacketBufferTest) ...
            TestSuite.fromClass(?NcsSensorTest) ...
            TestSuite.fromClass(?NcsControllerTest) ...
            TestSuite.fromClass(?NcsControllerWithFilterTest) ...
            TestSuite.fromClass(?EventBasedNcsSensorTest) ...
            TestSuite.fromClass(?EventBasedControllerTriggerCriterionTest) ...
            TestSuite.fromClass(?NcsPlantTest) ...
            TestSuite.fromClass(?NetworkedControlSystemTest) ...
            TestSuite.fromClass(?NetworkTypeTest) ...
            TestSuite.fromClass(?NcsTranslatorTest)
        ];

    results = [TestRunner.withTextOutput.run(tests) executeApiTests(runInParallel)];
end

