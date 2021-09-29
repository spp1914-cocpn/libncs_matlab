function config = BuildInvertedPendulumConfig(samplingInterval, massCart, massPendulum, friction, length, ...
    initialPlantState, plantNoiseCov, measNoiseCov)
    % Function to create a config struct with parameters for a discrete-time inverted pendulum 
    % (with uniform rod, i.e., the center of mass is located at the center of the rod) on a cart.
    % For the simulation of the pendulum, the nonlinear dynamics is used.
    % To use a linear model for the controller, the plant dynamics is linearized
    % around the unstable upward equilibrium which corresponds to the state
    % [0 0 pi 0]';
    %
    % Parameters:
    %   >> samplingInterval (Positive scalar)
    %      The sampling interval (in seconds) to be used for the discretization of the
    %      underlying continuous-time plant model.
    %
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
    %   >> initialPlantState (4-dimensional Vector or Distribution, optional)
    %      A 4-dimensional vector or a probability distribution denoting
    %      the initial integrator state, i.e., initial position and
    %      velocity of the cart, the initial deviation of the pendulum from the upward equalibrium and its 
    %      initial change.
    %      If a distribution is passed, the initial state is
    %      randomly drawn according to the given probability law.
    %      If left out, [0 0 pi+0.02 0]' is used as the initial plant state.
    %       
    %   >> plantNoiseCov (Positive definite matrix, optional)
    %      A positive definite 2-by-2 matrix, typically diagonal, the covariance matrix of the
    %      zero-mean Gaussian process noise (disturbance force acting on tip of pendulum rod and on cart, respectively)
    %      used for the nonlinear plant model and its linearization.
    %      If left out, a diagonal covariance matrix with 0.0001 * eye(2) is assumed.
    %
    %   >> measNoiseCov (Positive definite matrix, optional)
    %      A definite 2-by-2 matrix specifying the covarariance of the zero-mean Gaussian measurement noise that corrupts
    %      measurements taken at every time step.
    %      If left out, 0.0001 * eye(2) is used as default value.
    %
    % Returns:
    %   >> config (Struct)
    %      A configuration struct, equipped with the parameters as given to
    %      define an inverted pendulum plant.
    
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

    % state: % position of cart (1d), velocity of cart, deviation (angle/radiants) of
    % pendulum from equilibrium and angular velocity
    % measurement: position of cart and angle of pendulum
    % control input: force applied to the cart
    
    arguments
        samplingInterval (1,1) double {mustBePositive, mustBeFinite}
        massCart (1,1) double {mustBePositive, mustBeFinite} = 0.5; % mass of cart
        massPendulum (1,1) double {mustBePositive, mustBeFinite} = 0.2; % mass of pendulum
        friction (1,1) double {mustBePositive, mustBeFinite} = 0.1; % friction of the cart
        length (1,1) double {mustBePositive, mustBeFinite} = 0.3; % length of pendulum
        initialPlantState = [0; 0; pi+0.02; 0];
        plantNoiseCov (2,2) double {chol(plantNoiseCov)} = diag([1e-4, 1e-4]);
        measNoiseCov (2,2) double {chol(measNoiseCov)} = 0.001^2 * eye(2);
    end
    
    config.plant = InvertedPendulum(massCart, massPendulum, length, friction, samplingInterval);
    config.W_pend = plantNoiseCov;
    % data for the linearization
    [config.A, config.B, config.C, ~] = config.plant.linearizeAroundUpwardEquilibrium(samplingInterval);
    config.W = plantNoiseCov; % use the noise also for the linearized dynamics, processed inside initPlant() in ncs_initialize
    
    % use discrete-time noise for the nonlinear plant dynamics
    config.plant.setNoise(Gaussian(zeros(2, 1), plantNoiseCov));
    
    config.V = measNoiseCov;
    % we want to measure the deviation of the angle from equilibrium, but measure the angle
    % so add pi to the meas model used by the filter
    config.v_mean = [0 pi]';    
    
    config.sensor = LinearMeasurementModel(config.C);
    config.sensor.setNoise(Gaussian(zeros(2, 1), config.V));
    
    config.linearizationPoint = [0 0 pi 0]';
    config.samplingInterval = samplingInterval;
    config.initialPlantState = initialPlantState;

%     Q = diag([100 0 5000 0]); % state weighting matrix for LQR, small angle deviations are crucial
%     R = 100; % input weighting matrix for LQR
%     % this combination allows for larger inputs
%     Qd = integral(@(x) expm(contSysMatrix' * x) * Q * expm(contSysMatrix * x), 0, samplingInterval, 'ArrayValued', true)
%     Rd = integral(@(x) contInputMatrix' * integral(@(y) expm(contSysMatrix' * y), 0, x, 'ArrayValued', true) * Q * integral(@(y) expm(contSysMatrix' * y), 0, x, 'ArrayValued', true) * contInputMatrix + R, ...
%         0, samplingInterval, 'ArrayValued', true)    
end


