function config = BuildInvertedPendulumConfig(massCart, massPendulum, friction, length, ...
    samplingInterval, initialPlantState, plantNoiseCov, measNoiseCov)
    % Function to create a config struct with parameters for a discrete-time inverted pendulum 
    % (with uniform rod, i.e., the center of mass is located at the center of the rod) on a cart.
    % For the simulation of the pendulum, the deterministic, nonlinear dynamics is used.
    % To use a linear model for the controller, the plant dynamics is linearized
    % around the unstable upward equilibrium which corresponds to the state
    % [0 0 pi 0]';
    %
    % Parameters:
    %   >> massCart (Positive Scalar, optional)
    %      A positive scalar denoting the mass (in kg) of the cart.
    %      If left out, 0.5 kg is used as default value.
    % 
    %   >> massPendulum (Positive scalar, optional)
    %      A positive scalar denoting the mass (in kg) of the pendulum.
    %      If left out, 0.2 kg is used as default value.
    %
    %   >> friction (Positive scalar, optional)
    %      A positive scalar denoting the coefficient of friction for the cart (in Ns/m).
    %      If left out, 0.1 Ns/m is used as default value.
    %
    %   >> length (Positive scalar, optional)
    %      A positive scalar denoting the length of the pendulum rod (in m).
    %      If left out, 0.3 m is used as default value.
    %
    %   >> samplingInterval (Positive scalar, optional)
    %      The sampling interval (in seconds) to be used for the discretization of the
    %      underlying continuous-time plant model.
    %      If left out, 0.01 s is used as default value.
    %
    %   >> initialPlantState (4-dimensional Vector or Distribution)
    %      A 4-dimensional vector or a probability distribution denoting
    %      the initial integrator state, i.e., initial position and
    %      velocity of the cart, the initial deviation of the pendulum from the upward equalibrium and its 
    %      initial change.
    %      If a distribution is passed, the initial state is
    %      randomly drawn according to the given probability law.
    %      If left out, [0 0 pi+0.02 0]' is used as the initial plant state.
    %       
    %   >> plantNoiseCov (Positive definite matrix)
    %      A positive definite 4-by-4 matrix, the covariance matrix of the
    %      zero-mean Gaussian process noise used for the linearized plant model.
    %      If left out, a diagonal covariance matrix with [0.0001; 0.0001; 0] is assumed 
    %      for the linearized, continuous time model and then discretized.
    %
    %   >> measNoiseCov (Positive definite matrix)
    %      A definite 2-by-2 matrix specifying the covarariance of the zero-mean Gaussian measurement noise that corrupts
    %      measurements taken at every time step.
    %      If left out, 0.001^2 * eye(2) is used as default value. 
    %
    % Returns:
    %   >> config (Struct)
    %      A configuration struct, equipped with the parameters as given to
    %      define a double integrator plant.
    
    
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

    % state: % position of cart (1d), velocity of cart, deviation (angle/radiants) of
    % pendulum from equilibrium and angular velocity
    % measurement: position of cart and angle of pendulum
    % control input: force applied to the cart
    if nargin == 0
        massCart = 0.5; % mass of cart
        massPendulum = 0.2; % mass of pendulum
        friction = 0.1; % friction of the cart
    
        length = 0.3; % length of pendulum
        samplingInterval = 0.01; % 100Hz
        %samplingInterval = 0.001; % 1kHz
        initialPlantMean = [0; 0; pi+0.02; 0];
        initialPlantState = initialPlantMean;

        measNoiseCov = 0.001^2 * eye(2);
        sigmaPos = 0.001; % standard deviation of position
        sigmaDev = 0.001; % standard deviation of angle (deviation from equlibrium)
        % noise of the continuous-time lineraized model
        plantNoiseCov = diag([sigmaPos 0 sigmaDev 0]) .^ 2;
    elseif nargin ~= 9
        error('BuildInvertedPendulumConfig:InvalidNumberOfArgs', ...
            '** Invalid number of arguments (%d). Either pass none or all. **', nargin);
    end
    g = 9.81; % gravitational force
    inertia = massPendulum * length^2 / 3; % moment of inertia of the pendulum, assuming a uniform rod (center of mass in the middle)
    denom = inertia * (massCart + massPendulum) + massPendulum * length ^ 2 * massCart;
    contSysMatrix = [0 1 0 0;
                 0  -(inertia+massPendulum*length^2)*friction/denom ((massPendulum * length)^2*g)/denom 0;
                 0 0 0 1;
                 0 -(massPendulum*length* friction)/denom (massPendulum*length*(massPendulum + massCart)*g)/denom 0];
     
    contInputMatrix = [0;  (inertia+massPendulum*length^2)/denom; 0; (massPendulum*length)/denom];
    contMeasMatrix = [1 0 0 0; 0 0 1 0];
    
    contSys = ss(contSysMatrix, contInputMatrix, contMeasMatrix, 0); % no D matrix
    discreteSys = c2d(contSys, samplingInterval);
    
    config.A = discreteSys.A;
    config.B = discreteSys.B;
    config.C = discreteSys.C;
    config.samplingInterval = samplingInterval;
    config.initialPlantState = initialPlantState;
    % discretize the noise
    config.W = integral(@(x) expm(contSysMatrix*x) * plantNoiseCov * expm(contSysMatrix'*x), ...
        0, samplingInterval, 'ArrayValued', true);
    config.V = measNoiseCov;
    % we want to measure the deviation of the angle from equilibrium, but measure the angle
    % so add pi to the meas model used by the filter
    config.v_mean = [0 pi]';
    config.plant = InvertedPendulum(massCart, massPendulum, length, friction, samplingInterval);
    config.sensor = LinearMeasurementModel(config.C);
    config.sensor.setNoise(Gaussian(zeros(2, 1), config.V));
    
    config.linearizationPoint = [0 0 pi 0]';
end


