function config = CreateInvertedPendulumScenario(samplingInterval)
    % Function to create a config struct with parameters to define a
    % scenario where an inverted pendulum on a cart be controlled by a
    % controller that uses a linearized plant dynamics for simplicity.
    % Both measurements and control sequences are transmitted over networks. 
    % For simplicity, both networks have the same characteristics, i.e.,
    % they are described by the same delay probability distributions.
    %
    %
    % Parameters:
    %   >> samplingInterval (Positive scalar, optional)
    %      The sampling interval (in seconds) to be used for the discretization of the
    %      underlying continuous-time plant model.
    %      If left out, 0.01 s, which corresponds to a samplimng frequency
    %      of 100 Hz, is used as default value.
    %
    % Returns:
    %   >> config (Struct)
    %      A configuration struct, equipped with the specified
    %      parameters. 
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        samplingInterval (1,1) double {mustBePositive, mustBeFinite} = 0.01; % in seconds 
    end
    
    %0 time steps delay not possibly
    %1 85%
    %2-11 time steps uniform 7%
    %Inf 8%
    caDelayProbs = [1e-12 0.85  repmat(0.07/(12-2), 1, 12-2), 0.08]; % starting from zero delay
    caDelayProbs = caDelayProbs ./ sum(caDelayProbs);
    maxControlSequenceDelay = max(0, numel(caDelayProbs) - 1); % last entry indicates loss
    controlSequenceLength = 6;%numel(caDelayProbs);
    scDelayProbs = [];
    maxMeasDelay = maxControlSequenceDelay;
    
    filterClass = ?DelayedModeIMMF;
    controllerClass = ?InfiniteHorizonController;%?NominalPredictiveController; %?InfiniteHorizonController
    Q = diag([100 0 5000 0]); % state weighting matrix for LQR, small angle deviations are crucial
    R = 100; % input weighting matrix for LQR
    % this combination allows for larger inputs
    
    % discretize the cost matrices, to get equivalent discrete-time
%     % regulator
%     Q_disc = integral(@(tau) expm(A_cont' * tau) * Q_cont * expm(A_cont * tau), ...
%         0, samplingInterval, 'ArrayValued', true);
%     R_disc = integral(@(x) B_cont' * integral(@(y) expm(A_cont' * y), 0, x, 'ArrayValued', true) * Q_cont * integral(@(y) expm(A_cont * y), 0, x, 'ArrayValued', true) * B_cont + R_cont, ...
%         0, samplingInterval, 'ArrayValued', true);
    
    % small pendulum by default
    config = BuildInvertedPendulumConfig(samplingInterval);
    config = BuildNetworkConfig(maxMeasDelay, controlSequenceLength, ...
        maxControlSequenceDelay, caDelayProbs, scDelayProbs, config);
    
    config.networkType = NetworkType.UdpLikeWithAcks;
    % initial filter state
    % substract the equilibrium/linearization point
    plantStateCov = 0.01^2 * eye(4);
    initialEstimate = Gaussian(config.initialPlantState - config.linearizationPoint, plantStateCov);
    config = BuildControllerConfig(controllerClass.Name, Q, R, initialEstimate, filterClass.Name, config);
    
    %config.qocRateFunc = CreateFitShortPendulum(false);
    
%     config.stateConstraints = [pi/36;pi/36]; % radians, max deviation from equilibrium (5 degrees)
%     config.stateConstraintWeightings = [0 0;    
%                                         0 0;
%                                         1 -1;
%                                         0 0];
%     config.inputConstraints = 0;
%     config.inputConstraintWeightings = 0; % dummy constraints on the input
end

