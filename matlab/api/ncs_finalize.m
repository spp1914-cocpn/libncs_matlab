function [costs, stats] = ncs_finalize(ncsHandle)
    % Finish a control task (NCS simulation) in Matlab and clean up resources.
    %
    % Parameters:
    %   >> ncsHandle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem.
    %
    % Returns:
    %   << costs (Nonnegative scalar)
    %      The accrued costs according to the controller's underlying cost functional.
    %
    %   << stats (Struct)
    %     Struct containing statistical data gathered during the execution
    %     of the control task. At least the following fields are present:
    %     -numUsedMeasurements (Row vector of nonnegative integers), indicating the number
    %     of measurements that have been used by the controller/estimator at each time step
    %     -numDiscardedMeasurements (Row vector of nonnegative integers), indicating the number
    %     of measurements that have been discarded by the controller/estimator at each time step
    %     -appliedInputs (Matrix), 
    %     containing the actually applied control inputs per time step
    %     -trueStates (Matrix), 
    %     containing the true plant states per time step
    %     -controllerStates (Matrix), 
    %     containing the plant states kept by the controller per time step
    %     -numDiscardedControlSequences (Row vector of nonnegative integers), indicating the number
    %     of received control sequences that have been discarded by the actuator at each time step
    
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
    
    ncs = GetNcsByHandle(ncsHandle);

    ncsStats = ncs.getStatistics();
    costs = ncs.computeTotalControlCosts();
    stats.numUsedMeasurements = ncsStats.numUsedMeasurements;
    stats.numDiscardedMeasurements = ncsStats.numDiscardedMeasurements;
    stats.trueStates = ncsStats.trueStates;
    stats.controllerStates = ncsStats.controllerStates;
    stats.appliedInputs = ncsStats.appliedInputs;
    stats.numDiscardedControlSequences = ncsStats.numDiscardedControlSequences;        
   
    ComponentMap.getInstance().removeComponentByIndex(ncsHandle);
end

