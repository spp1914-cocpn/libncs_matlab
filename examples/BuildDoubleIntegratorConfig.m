function config = BuildDoubleIntegratorConfig(mass, samplingInterval, initialPlantState, plantNoiseCov, measNoiseCov)
    % Function to create a config struct with parameters for a stochastic, discrete-time double integrator plant.
    %
    % Parameters:
    %   >> mass (Positive Scalar, optional)
    %      A positive scalar denoting the mass (in kg) to be moved.
    %      If left out, 1 kg is used as default value.
    % 
    %   >> samplingInterval (Positive scalar, optional)
    %      The sampling interval (in seconds) to be used for the discretization of the
    %      underlying continuous-time plant model.
    %      If left out, 0.01 s is used as default value.
    %
    %   >> initialPlantState (2-dimensional Vector or Distribution)
    %      A 2-dimensional vector or a probability distribution denoting
    %      the initial integrator state, i.e., initial position and
    %      velocity.
    %      If a distribution is passed, the initial state is
    %      randomly drawn according to the given probability law.
    %      If left out, a Gaussian with mean [100; 0] and covariance matrix 0.5 * eye(2) is used as default value.
    %       
    %   >> plantNoiseCov (Positive definite matrix)
    %      A positive definite 2-by-2 matrix, the covariance matrix of the
    %      zero-mean Gaussian process noise.
    %      If left out, 0.1 * eye(2) is used as default value.
    %
    %   >> measNoiseCov (Nonnegative Scalar)
    %      A positive scalar denoting the variance of the zero-mean Gaussian measurement noise that corrupts the
    %      position measurements taken at every time step.
    %      If left out, a variance of 0.04 is used as default value. 
    %
    % Returns:
    %   >> config (Struct)
    %      A configuration struct, equipped with the parameters as given to
    %      define a double integrator plant.
    
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

    % state: % position of mass (1d), velocity of mass
    % measurement: position is directly accessible
    % control input: force applied to accelerate the mass
    if nargin == 0
        mass = 1; % kg
        samplingInterval = 0.01; % sec, 100 Hz
        initialPlantMean = [100; 0];
        initialPlantCov = 0.5 * eye(2);
        initialPlantState = Gaussian(initialPlantMean, initialPlantCov);
        plantNoiseCov = 0.1 * eye(2);
        measNoiseCov = 0.2^2;
    elseif nargin ~= 5
        error('BuildDoubleIntegratorConfig:InvalidNumberOfArgs', ...
            '** Invalid number of arguments (%d). Either pass none or all. **', nargin);
    end
    contSysMatrix = [0 1; 0 0];
    contInputMatrix = [0; 1 / mass];
    contMeasMatrix = [1 0];
    
    contSys = ss(contSysMatrix, contInputMatrix, contMeasMatrix, 0); % no D matrix
    discreteSys = c2d(contSys, samplingInterval);
    
    config.A = discreteSys.A;
    config.B = discreteSys.B;
    config.C = discreteSys.C;
    config.samplingInterval = samplingInterval;
    config.initialPlantState = initialPlantState;
    config.W = plantNoiseCov;
    config.V = measNoiseCov;
end

