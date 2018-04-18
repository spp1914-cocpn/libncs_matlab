function config = CreateDoubleIntegratorScenario()
    % Function to create a config struct with parameters to define a
    % scenario where a double integrator plant shall be driven to the origin by a
    % FiniteHorizonController and where both measurements and control sequence are transmitted over networks. 
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

    %0 time steps delay not possibly
    %1 85%
    %2-11 time steps uniform 7%
    %Inf 8%
    caDelayProbs = [1e-6 0.85  repmat(0.07/(12-2), 1, 12-2), 0.08];
    caDelayProbs = caDelayProbs ./ sum(caDelayProbs);

    controlSequenceLength = numel(caDelayProbs);
    %scDelayProbs = caDelayProbs;
    %maxMeasDelay = max(0, numel(scDelayProbs) - 2);% last entry indicates loss
    scDelayProbs = [];
    maxMeasDelay = max(0, numel(caDelayProbs) - 1); % last entry indicates loss
    
    filterClass = ?DelayedModeIMMF;
    controllerClass = ?NominalPredictiveController;
    Q = eye(2); 
    R = 1; 
    maxControlSequenceDelay = Inf;
    
    config = BuildDoubleIntegratorConfig();
    config = BuildNetworkConfig(maxMeasDelay, controlSequenceLength, ...
        maxControlSequenceDelay, caDelayProbs, scDelayProbs, config);
    
    config.networkType = NetworkType.UdpLikeWithAcks;
    
    initialPlantCov = 0.5 * eye(2); % taken from ACC 2013 paper by Jörg and Maxim
    initialEstimate = Gaussian(config.initialPlantState, initialPlantCov);
    config = BuildControllerConfig(controllerClass.Name, Q, R, initialEstimate, filterClass.Name, config);
end

