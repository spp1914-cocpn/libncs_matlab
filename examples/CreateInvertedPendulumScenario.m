function config = CreateInvertedPendulumScenario()
    % Function to create a config struct with parameters to define a
    % scenario where an inverted pendulum on a cart whose dynamics has been linearized around the 
    % upward equlibrium shall be controlled.
    % Both measurements and control sequence are transmitted over networks. 
    % For simplicity, both networks have the same characteristics, i.e.,
    % they are described by the same delay probability distributions.
    %
    % Returns:
    %   >> config (Struct)
    %      A configuration struct, equipped with the specified
    %      parameters. 
    
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

    % configuration mainly based on Fusion 13 paper by Jörg Fischer
    caDelayProbs = [0.001 0.019 0.08 0.14 0.17 0.17 0.14 0.08 0.019 0.001 0.18]; % starting from zero delay
    maxControlSequenceDelay = max(0, numel(caDelayProbs) - 2); % last entry indicates loss
    controlSequenceLength = numel(caDelayProbs);
    scDelayProbs = [0.001 0.019 0.08 0.14 0.17 0.17 0.14 0.08 0.019 0.001 0.18];
    maxMeasDelay = max(0, numel(scDelayProbs) - 2);% last entry indicates loss
    
    filterClassName = 'DelayedKF';
    controllerClassName = 'NominalPredictiveController';
    Q = diag([5000 0 100 0]); % state weighting matrix for LQR
    R = 100; % input weighting matrix for LQR
    
    config = BuildInvertedPendulumConfig();
    config = BuildNetworkConfig(maxMeasDelay, controlSequenceLength, ...
        maxControlSequenceDelay, caDelayProbs, scDelayProbs, config);
    % we use a TCP-like network here
    config.networkType = NetworkType.TcpLike;
    config = BuildFilterControllerConfig(filterClassName, config.initialPlantState, Q, R, controllerClassName, config);
end

