function config = CreateDoubleIntegratorScenario()
    % Function to create a config struct with parameters to define a
    % scenario where a double integrator plant (with unit mass), subject to a disturbance force, shall be driven to the origin
    % and where both measurements and control sequences are transmitted over networks. 
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

    mass = 1; % kg
    samplingInterval = 0.01; % sec, 100 Hz
    initialPlantState = [0.1; 0.01];
    % intensity (PSD) of the zero-mean Gaussian (continuous-time) disturbance force acting on the mass (i.e., process noise)
    % the disturbance force enters the plant in the same way the controlled
    % force does
    disturbanceForceVar = 1;
    % variance of the zero-mean Gaussian measurement noise that corrupts the
    % position measurements taken at every time step.
    measNoiseVar = 1e-6;
    
    % setup continuous-time dynamics
    A_cont = [0 1; 0 0];
    B_cont = [0; 1 / mass];
    C_cont = [1 0];
    
    contSys = ss(A_cont, B_cont, C_cont, 0); % no D matrix
    discreteSys = c2d(contSys, samplingInterval);
 
    config.A = discreteSys.A;
    config.B = discreteSys.B;
    config.C = discreteSys.C;
    config.samplingInterval = samplingInterval;
    config.initialPlantState = initialPlantState;
    % we must discretize the process noise
    % assume that the disturbance force enters the plant in the same way
    % the controlled force does
    % that is, we must compute a (controllability) Gramian-like expression
        %config.W = integral(@(tau) expm(A_cont * tau) * [0 0; 0 disturbanceForceVar/(mass ^ 2)] * expm(A_cont' * tau), ...
        %    0, samplingInterval, 'ArrayValued', true);
    % or use the following trick due to van Loan
    F = [-A_cont [0 0; 0 disturbanceForceVar/(mass ^ 2)]; zeros(2) A_cont'] * samplingInterval;
    G = expm(F);
    config.W = config.A * G(1:2, 3:4);
    config.V = measNoiseVar;
    
    %0 time steps delay not possibly
    %1 85%
    %2-11 time steps uniform 7%
    %Inf 8%
    caDelayProbs = [1e-6 0.85  repmat(0.07/(12-2), 1, 12-2), 0.08];
    caDelayProbs = caDelayProbs ./ sum(caDelayProbs);

    controlSequenceLength = 6;%numel(caDelayProbs);
    %scDelayProbs = caDelayProbs;
    %maxMeasDelay = max(0, numel(scDelayProbs) - 2);% last entry indicates loss
    scDelayProbs = [];
    maxMeasDelay = max(0, numel(caDelayProbs) - 1); % last entry indicates loss
    
    filterClass = ?DelayedModeIMMF; % not needed with the given controller
    controllerClass = ?IMMBasedRecedingHorizonController;
    
    Q_cont = [1560 0; 0 0]; % continuous-time weighting matrix for state
    R_cont = 1;  % continuous-time weighting matrix for input
    % the weighting matrices are selected such that if a continuous-time LQR
    % was used, the system would have closed-loop poles a 1 Hz and damping ratio of
    % D = 0.7
    
    % discretize the cost matrices, to get equivalent discrete-time
    % regulator
    Q_disc = integral(@(tau) expm(A_cont' * tau) * Q_cont * expm(A_cont * tau), ...
        0, samplingInterval, 'ArrayValued', true);
    R_disc = integral(@(x) B_cont' * integral(@(y) expm(A_cont' * y), 0, x, 'ArrayValued', true) * Q_cont * integral(@(y) expm(A_cont * y), 0, x, 'ArrayValued', true) * B_cont + R_cont, ...
        0, samplingInterval, 'ArrayValued', true);
    
    maxControlSequenceDelay = Inf;
        
    config = BuildNetworkConfig(maxMeasDelay, controlSequenceLength, ...
        maxControlSequenceDelay, caDelayProbs, scDelayProbs, config);
    
    config.networkType = NetworkType.UdpLikeWithAcks;
    config.plant = LinearPlant(config.A, config.B, config.W);
    
    initialPlantCov = 0.5 * eye(2); % taken from ACC 2013 paper by Jörg and Maxim
    initialEstimate = Gaussian(config.initialPlantState, initialPlantCov);
    config = BuildControllerConfig(controllerClass.Name, Q_disc, R_disc, initialEstimate, filterClass.Name, config);

end

