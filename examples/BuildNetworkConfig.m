function configOut = BuildNetworkConfig(maxMeasDelay, controlSequenceLength, maxControlSequenceDelay, ...
    caDelayProbs, scDelayProbs, config)
    % Function to create a config struct with network-related parameters as required by ncs_initialize.
    %
    % Parameters:
    %   >> maxMeasDelay (Nonnegative Integer)
    %      A nonnegative integer denoting the maximum number of time steps a measurement
    %      may experience before in order to be processed by the NCS components. 
    % 
    %   >> controlSequenceLength (Positive Integer)
    %      The sequence length to be employed by the sequence-based controller in order to cope with packet delays.
    %
    %   >> maxControlSequenceDelay (Nonnegative Integer)
    %      A nonnegative integer denoting the maximum number of time steps
    %      a control sequence may experience before in order to be processed by the NCS components.
    %       
    %   >> caDelayProbs (Nonnegative Vector)
    %      A vector with nonnegative entries summing up to 1 denoting the probability distribution
    %      of the delays in the CA-link (network between controller and
    %      actuator/plant).
    %
    %   >> scDelayProbs (Nonnegative Vector, optional)
    %      A vector with nonnegative entries summing up to 1 denoting the probability distribution
    %      of the delays in the SC-link (network between sensor and
    %      controller).
    %
    %   >> config (Struct, optional)
    %      A structure containing configuration parameters which shall be extended. 
    %      If this parameter is left out, a new structure with network-related parameters is created instead.
    %
    % Returns:
    %   >> configOut (Struct)
    %      A configuration struct, equipped with network-related
    %      parameters. If a config is passed as las input parameter, it is
    %      returned here.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2018  Florian Rosenthal<florian.rosenthal@kit.edu>
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

    if nargin == 6
        configOut = config;
    elseif nargin == 4
        scDelayProbs = [];
    end
    
    configOut.maxMeasDelay = maxMeasDelay;
    configOut.controlSequenceLength = controlSequenceLength;
    configOut.maxControlSequenceDelay = maxControlSequenceDelay;
    configOut.caDelayProbs = caDelayProbs;

    if ~isempty(scDelayProbs)
        configOut.scDelayProbs = scDelayProbs;
    end
end

