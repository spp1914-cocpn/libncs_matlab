function configOut = BuildControllerConfig(controllerClassName, Q, R, initialEstimate, filterClassName, config)
    % Function to create a config struct with parameters to define an
    % NcsController, i.e, a controller with quadratic cost function.
    %
    % Parameters:
    %   >> controllerClassName (Character Array)
    %      A character containg the name of the controller class (i.e., a subclass of SequenceBasedController) to be utilized. 
    %
    %   >> Q (Positive semi-definite matrix)
    %      The state weighting matrix in the controller's underlying cost function.
    %
    %   >> R (Positive definite matrix)
    %      The input weighting matrix in the controller's underlying cost function.
    %
    %   >> initialEstimate (Distribution, optional)
    %      A Distribution (e.g., a Gaussian or GaussianMixture) specifying
    %      the intial state of the controller or the filter associated with
    %      it.
    %      Leave out or pass the empty matrix, if not required.
    %      Should, however, be present, if filter is utilized.
    %
    %   >> filterClassName (Character Array, optional)
    %      A character containg the name of the filter class (i.e., a subclass of DelayedMeasurementsFilter) to be utilized. 
    %      Leave out or pass the empty matrix, in case none shall be
    %      utilized.
    %
    %   >> config (Struct, optional)
    %      A structure containing configuration parameters which shall be extended. 
    %      If this parameter is left out, a new structure with the specified parameters is created instead.
    %
    % Returns:
    %   >> configOut (Struct)
    %      A configuration struct, equipped with the specified
    %      parameters. If a config is passed as las input parameter, it is
    %      returned here.
    
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

    if nargin == 6
        configOut = config;
    elseif nargin == 3
        filterClassName = [];
        initialEstimate = [];
    elseif nargin == 4
        filterClassName = [];
    end
    
    configOut.controllerClassName = controllerClassName;
    configOut.Q = Q;
    configOut.R = R;
    if ~isempty(filterClassName)
        configOut.filterClassName = filterClassName;
    end
    if ~isempty(initialEstimate)
        configOut.initialEstimate = initialEstimate;
    end
end

