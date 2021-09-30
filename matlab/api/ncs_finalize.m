function [costs, controllerStats, plantStats] = ncs_finalize(ncsHandle)
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
    %   << controllerStats (Struct)
    %     Struct containing control-related data gathered during the execution
    %     of the control task. At least the following fields are present:
    %     -numUsedMeasurements (Row vector of nonnegative integers), indicating the number
    %     of measurements that have been used by the controller/estimator at each time step
    %     -numDiscardedMeasurements (Row vector of nonnegative integers), indicating the number
    %     of measurements that have been discarded by the controller/estimator at each time step
    %     -controllerStates (Matrix), 
    %     containing the plant states kept by the controller per time step
    %     -numDiscardedControlSequences (Row vector of nonnegative integers), indicating the number
    %     of received control sequences that have been discarded by the actuator at each time step
    %     -controllerTimes (Row vector of nonnegative scalars), the i-th
    %     entry indicates the simulation time (in seconds) corresponding to
    %     the i-th controller time step
    %     -trueModes (Row vector of positive integers within [1, N+1]), where N is the employed sequence length indicating the the
    %     active mode theta_k (one-based) in the augmented dynamical system (MJLS) describing plant and
    %     actuator; value=i, i=1,..,N, equals age-1 of the used sequence, and value=N+1 indicates application of default input
    %
    %   << plantStats (Struct)
    %     Struct containing plant-related data gathered during the execution
    %     of the control task. At least the following fields are present:
    %     -appliedInputs (Matrix), 
    %     containing the actually applied control inputs u per time step (time step w.r.t
    %     plant sampling rate)
    %     -trueStates (Matrix), 
    %     containing the true plant states x per time step (time step w.r.t
    %     plant sampling rate)
    %     
    %     If the task was to stabilize a double inverted pendulum (i.e.,
    %     the plant was DoubleInvertedPendulum instance), additionally the
    %     following field is present:
    %     -trueStateNorms (Row vector of nonnegative scalars), the i-th
    %     entry indicates the Euclidean norm ||x|| of the true plant state x
    %     at time step i (time step w.r.t plant sampling rate)
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
    controllerStats.numUsedMeasurements = ncsStats.numUsedMeasurements;
    controllerStats.numDiscardedMeasurements = ncsStats.numDiscardedMeasurements;
    controllerStats.numDiscardedControlSequences = ncsStats.numDiscardedControlSequences;        
    controllerStats.controllerStates = ncsStats.controllerStates;
    controllerStats.controllerTimes = ncsStats.times'; % transpose into row vector
    controllerStats.trueModes = ncsStats.trueModes;
    
    plantStats.trueStates = ncsStats.trueStates;
    plantStats.appliedInputs = ncsStats.appliedInputs;
    if isfield(ncsStats, 'trueStateNorms')
        plantStats.trueStateNorms = ncsStats.trueStateNorms;
    end
      
    ComponentMap.getInstance().removeComponentByIndex(ncsHandle);
end

