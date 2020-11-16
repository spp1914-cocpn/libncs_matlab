function handle = ncs_initialize(maxSimTime, id, configStruct, filename)
    % Initialize a control task (NCS simulation) in Matlab.
    %
    % Parameters:
    %   >> maxSimTime (Positive Scalar)
    %      A positive scalar denoting the simulation time in pico-seconds. 
    % 
    %   >> id
    %      Name or identifier for the NCS.
    %
    %   >> configStruct (Struct or empty matrix)
    %      A structure containing configuration parameters to complement or override 
    %      those defined in the corresponding config file given by
    %      <filename>. If the empty matrix is passed here, all
    %      configuration parameters are read from the configuration file.
    %       
    %   >> filename (Character Vector)
    %      A character vector containing the full filepath of the config
    %      file (with extension .mat).
    %
    % Returns:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies the initialized NetworkedControlSystem.
    
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
    
    persistent requiredVariables;
    requiredVariables = {'controlSequenceLength', 'maxMeasDelay', ...
        'caDelayProbs', 'A', 'B', 'C', 'W', 'Q', 'R', 'plant', ...
        'initialPlantState', 'samplingInterval', 'plantSamplingInterval', 'controllerClassName'};
    % optional variables:
    %   - scDelayProbs: specifies the true or assumed distribution of the delays in the
    %     network between sensor and controller (i.e., for measurements);
    %     required by some controllers or filters (nonnegative vector) 
    %   - filterClassName: if a filter is required for state estimation
    %   - initialEstimate: the initial controller state or the initial
    %     estimate of the employed filter (if any)
    %   - networkType: specifies the network in use (NetworkType) (default UdpLikeWithAcks)
    %   - Z: performance output matrix for the plant output z_k = Z * x_k
    %   - refTrajectory: reference trajectory z_ref to track with the plant output z_k
    %   - V: covariance matrix of the additive, Gaussian measurement noise assumed by the controllers/filters, if present (Positive definite matrix)
    %   - v_mean: mean of the measurement noise/offset, if present (default zero is assumed otherwise)
    %   - sensor: linear dynamics to be used for simulation of the sensor(LinearMeasurementModel)
    %   - sensorEventBased: flag to indicate whether the sensor shall
    %     transmit measurements in an event-based manner (default false)
    %   - sensorMeasDelta: the threshold value (delta) for the decision rule
    %     if the sensor employs a send-on-delta strategy (nonnegative scalar)
    %   - controllerEventBased: flag to indicate whether the controller shall
    %     transmit sequences in an event-based manner (by using deadband control) (default false)
    %   - controllerDeadband: the threshold value (deadband) for decison rule the
    %     if the controller employs a deadband control strategy (nonnegative scalar)
    %   - controllerEventTrigger: the event trigger to be used 
    %     if the controller employs a deadband control strategy;
    %     Possible values are given by the EventBasedControllerTriggerCriterion enum:
    %     EventBasedControllerTriggerCriterion.ControlError (or 1)
    %     EventBasedControllerTriggerCriterion.StageCosts (or 2)
    %     EventBasedControllerTriggerCriterion.Sequence (or 3) 
    %     It is also possible to pass the corresponding ids instead.
    %     The default value is EventBasedControllerTriggerCriterion.Sequence
    %   - linearizationPoint: if the controller uses a linear approximation
    %     of the nonlinear plant dynamics, the linearization point should
    %     be provided (vector)
    %   - transmissionCosts: the costs for transmitting a control sequence
    %   - stateConstraints: linear constraints b_i for MPC, i.e., a_i'*x_k <= b_i
    %   - stateConstraintWeightings: linear constraints weightings a_i for MPC, i.e., a_i'*x_k <= b_i
    %   - inputConstraints: linear constraints d_i for MPC, i.e., c_i'*u_k <= d_i
    %   - inputConstraintWeightings: linear constraints weightings c_i for MPC, i.e., c_i'*u_k <= d_i
    %   - controlErrorWindowSize: the control error is in integral measure,
    %     this parameter specifies the size (in time steps) of the sliding window used to
    %     compute it (positive integer)
    %   - ignoreControllerCache: flag to indicate whether a previously
    %     computed and saved/cached controller shall be ignored (if
    %     available) (default true)
    %   - mpcHorizon: if the specified controller is a receding horizon one
    %     (i.e., a model predictive controller), the value of this variable
    %     denotes the length of the prediction horizon (positive integer);
    %     if not specified, the employed sequence length is used by default
    
    assert(isempty(configStruct) || (isstruct(configStruct) && isscalar(configStruct)), ...
        'ncs_initialize:InvalidConfigStruct', ...
        '** <configStruct> must be a single struct **');    
    assert(ischar(filename) && isvector(filename), ...
        'ncs_initialize:InvalidFilename', ...
        '** <filename> must be a character vector **');
    assert(Checks.isPosScalar(maxSimTime), ...
        'ncs_initialize:InvalidMaxSimeTime', ...
        '** <maxSimTime> must be positive and given in pico-seconds. **');
    
    [pathStr, configFileName, extension] = fileparts(filename);
    % 2 is returned in case of existing file
    assert(exist(filename, 'file') == 2 && strcmp(extension, '.mat'), ...
        'ncs_initialize:InvalidFile', ...
        '** %s does not exist or is not a mat-file **', filename);
        
    if isempty(configStruct)
        config = load(filename);
    else
        % combine data from config file and from struct into a single config struct
        config = configStruct;
        tmp = load(filename);
        fields = fieldnames(tmp);
        notFoundIdx = find(~isfield(config, fields))'; % row vector
        for i=notFoundIdx
            name = fields{i};
            % append config value from the mat file
            config.(name) = tmp.(name);
        end
        clear('tmp');
    end

    checkConfig(config, requiredVariables);
    %now ensure that all scalar, integers are internally represented as double
    allFields = fieldnames(config);
    for i=1:length(allFields)
        name = allFields{i};
        if ~isequal(name, 'networkType') && ~isequal(name, 'controllerEventTrigger') ...
                && isscalar(config.(name)) && isinteger(config.(name))
            % cast to double            
            config.(name) = double(config.(name));
        end
    end
        
    % ensure that maxSimTime is always a double, so that ConvertToSeconds
    % can return a fractional value
    loopSteps = floor(ConvertToSeconds(double(maxSimTime)) / config.samplingInterval);
    
    [plant, config] = initPlant(config);
    actuator = initActuator(config);
    ncsPlant = NcsPlant(plant, actuator);

    if ~getConfigValueOrDefault(config, 'ignoreControllerCache', true)
        % try to reuse controller, if possible
        controller = doControllerCacheLookup(config, configFileName, pathStr, loopSteps);
        if isempty(controller)
            controller = initController(config, loopSteps);
            newCacheInfo = Cache.insert(controller, [configFileName '_controller_' int2str(loopSteps)]);
            if ~isempty(newCacheInfo)
                % controller was successfully saved/cached
                writeCacheInfo(configFileName, pathStr, newCacheInfo);
            end
        end
    else
        % compute the controller
        controller = initController(config, loopSteps);
    end
        
    sensor = initSensor(config);
    if getConfigValueOrDefault(config, 'sensorEventBased', false)
        ncsSensor = EventBasedNcsSensor(sensor, ...
            getConfigValueOrDefault(config, 'sensorMeasDelta', EventBasedNcsSensor.defaultMeasurementDelta));            
    else
        ncsSensor = NcsSensor(sensor);
    end
    
    if controller.requiresExternalStateEstimate
        % we need a filter
        assert(isfield(config, 'filterClassName'), ...
            'ncs_initialize:FilterClassNameMissing', ...
            ['** Variable <filterClassName> must be present in the configuration to init a filter/estimator '...
            'required by %s **'], class(controller));
        ncsController = initNcsControllerWithFilter(config, controller, actuator);
    else
        % controller does not require an external filter
        ncsController = initNcsController(config, controller, actuator);
    end    
    
    ncsController.controlErrorWindowSize = getConfigValueOrDefault(config, 'controlErrorWindowSize', ...
        NcsController.defaultControlErrorWindowSize);
     
    % init NCS: check if network type was specified
    ncs = NetworkedControlSystem(ncsController, ncsPlant, ncsSensor, id, config.samplingInterval, ...
        config.plantSamplingInterval, getConfigValueOrDefault(config, 'networkType', NetworkType.UdpLikeWithAcks));
    
    ncs.initPlant(config.initialPlantState);
    ncs.initStatisticsRecording(maxSimTime);
    
    if isfield(config, 'qocRateFunc') && isfield(config, 'controlErrorQocFunc')
        translator = NcsTranslator(config.qocRateFunc, config.controlErrorQocFunc, 1 / ncs.samplingInterval);
        %# function cfit sfit
        ncs.attachTranslator(translator);
    elseif isfield(config, 'translator')
        % the following pragma is required to ensure deployed code is runnable
        %# function cfit sfit
        ncs.attachTranslator(config.translator);
    end
    
    handle = ComponentMap.getInstance().addComponent(ncs);
    clear('config');
end

%% initNcsControllerWithFilter
function ncsControllerWithFilter = initNcsControllerWithFilter(config, controller, actuator)
    % we need a filter
    [filter, filterPlantModel, filterSensorModel] = initFilter(config);    
    if getConfigValueOrDefault(config, 'controllerEventBased', false)
        % event-based controller
        defaultLinearizationPoint = [];
        ncsControllerWithFilter = EventBasedNcsControllerWithFilter(controller, filter, ...
                filterPlantModel, filterSensorModel, actuator.defaultInput, config.caDelayProbs, ...
                getConfigValueOrDefault(config, 'linearizationPoint', defaultLinearizationPoint));
        
        % set the deadband if provided (else default value is used)
        ncsControllerWithFilter.deadband = getConfigValueOrDefault(config, 'controllerDeadband', ...
            EventBasedNcsControllerWithFilter.defaultDeadband);
        % set the event trigger, if provided (else default value is used)
        ncsControllerWithFilter.eventTrigger = uint8(getConfigValueOrDefault(config, 'controllerEventTrigger', ...
            EventBasedControllerTriggerCriterion.Sequence));        
    else
        % linearization point might be given
        ncsControllerWithFilter = NcsControllerWithFilter(controller, filter, ...
            filterPlantModel, filterSensorModel, actuator.defaultInput, config.caDelayProbs, ...
            getConfigValueOrDefault(config, 'linearizationPoint', []));
    end
end

%% initNcsController
function ncsController = initNcsController(config, controller, actuator)
    % controller does not require an external filter
    if getConfigValueOrDefault(config, 'controllerEventBased', false)
        % event-based controller
        ncsController = EventBasedNcsController(controller, actuator.defaultInput, ...
            getConfigValueOrDefault(config, 'linearizationPoint', []));
        % set the deadband, if provided (else default value is used)
        ncsController.deadband = getConfigValueOrDefault(config, 'controllerDeadband', ...
            EventBasedNcsControllerWithFilter.defaultDeadband);
        % set the event trigger, if provided (else default value is used)
        ncsController.eventTrigger = uint8(getConfigValueOrDefault(config, 'controllerEventTrigger', ...
            EventBasedControllerTriggerCriterion.Sequence)); 
    else
        ncsController = NcsController(controller, actuator.defaultInput, ...
            getConfigValueOrDefault(config, 'linearizationPoint', []));
    end    
end

%% doControllerCacheLookup
function controller = doControllerCacheLookup(config, configFileName, pathStr, loopSteps)
    cacheInfo = readCacheInfo(configFileName, pathStr);
    controller = [];
    if ~isempty(cacheInfo)
        cachedController = Cache.lookup(cacheInfo);
        if ~isempty(cachedController) && strcmp(class(cachedController), config.controllerClassName)
            % check for additional, type dependent properties that must
            % match
            switch config.controllerClassName
                case 'FiniteHorizonController'
                    % we assume that constraints did not change
                    if cachedController.horizonLength == loopSteps ...
                            && cachedController.sequenceLength == config.controlSequenceLength
                        controller = cachedController;
                    end
                case 'FiniteHorizonTrackingController'
                    if cachedController.horizonLength == loopSteps ...
                            && cachedController.sequenceLength == config.controlSequenceLength ...
                            && isequal(config.refTrajectory, cachedController.refTrajectory)
                        controller = cachedController;
                    end
                case 'NominalPredictiveController'
                    % ensure that we use the correct sequence length
                    cachedController.changeSequenceLength(config.controlSequenceLength);                    
                    if ~isequal(cachedController.Q, config.Q) || ~isequal(cachedController.R, config.R)
                        cachedController.changeCostMatrices(config.Q, config.R);
                    end
                    controller = cachedController;
                case 'LinearlyConstrainedPredictiveController'
                    refTrajectory = [];
                    if isfield(config, 'refTrajectory')
                        refTrajectory = config.refTrajectory;
                    end
                    % so far, reference trajectory must be equal
                    if isequal(refTrajectory, cachedController.refTrajectory)
                        controller = cachedController;
                        if controller.sequenceLength ~= config.controlSequenceLength
                            % use sequence length also as length of
                            % optimization horizon
                            controller.changeHorizonLength(config.controlSequenceLength);
                            controller.changeSequenceLength(config.controlSequenceLength);
                        end
                        % check if the constraints changed
                        [stateConstraints, weightings] = controller.getStateConstraints();
                        if ~isequal(stateConstraints, config.stateConstraints) ...
                                || ~isequal(weightings, config.stateConstraintWeightings)
                            controller.changeStateConstraints(config.stateConstraintWeightings, ...
                                config.stateConstraints);
                        end
                        [inputConstraints, weightings] = controller.getInputConstraints();
                        if ~isequal(inputConstraints, config.inputConstraints) ...
                                || ~isequal(weightings, config.inputConstraintWeightings)
                            controller.changeInputConstraints(config.inputConstraintWeightings, ...
                                config.inputConstraints);
                        end
                    end
                case 'InfiniteHorizonController'
                    if cachedController.sequenceLength == config.controlSequenceLength
                        controller = cachedController;
                    end
                case 'InfiniteHorizonUdpLikeController'
                    if cachedController.sequenceLength == config.controlSequenceLength ...
                            && cachedController.maxMeasurementDelay == config.maxMeasDelay
                        controller = cachedController;
                    end
                case 'EventTriggeredInfiniteHorizonController'
                    if cachedController.sequenceLength == config.controlSequenceLength
                        controller = cachedController;
                        controller.transmissionCosts = config.transmissionCosts;                        
                    end
                otherwise
                    controller = cachedController;
            end
         end
    end
end

%% initActuator
function actuator = initActuator(config)
    actuator = BufferingActuator(config.controlSequenceLength, zeros(size(config.B,2), 1));
end

%% initPlant
function [plant, configOut] = initPlant(configIn)
    configOut = configIn;
    switch class(configIn.plant)
        case 'InvertedPendulum'
            plant = configIn.plant;
            % set the noise of the continuous-time linearization if present
            plant.W_cont = getConfigValueOrDefault(configIn, 'W_cont', zeros(4));
            
            % obtain discretized and linearized model used by controller
            [configOut.A, configOut.B, configOut.C, configOut.W] = plant.linearizeAroundUpwardEquilibrium(configIn.samplingInterval);

            % plant is simulated with sampling rate given in config
            plant.samplingInterval = configIn.plantSamplingInterval;
            if getConfigValueOrDefault(configIn, 'usePlantNoise', true)
                % use the discrete-time noise also for simulation the nonlinear plant
                % use discretization obtained using plant sampling rate
                % based on value of W_cont
                [~, ~, ~, W_d] = plant.linearizeAroundUpwardEquilibrium();
                plant.setNoise(Gaussian(zeros(4, 1), W_d));
            end
%             % we also translate the cost function for a fair comparison
%             Q_d = integral(@(x) expm(A_cont'*x) * configIn.Q * expm(A_cont*x), ...
%                 0, configIn.samplingInterval, 'ArrayValued', true)
%             R_d = configIn.samplingInterval * configIn.R + B_cont' * integral(@(tau) integral(@(s) expm(A_cont'*s), 0, tau, 'ArrayValued', true) * configIn.Q * integral(@(s) expm(A_cont*s), 0, tau, 'ArrayValued', true), ...
%                 0, configIn.samplingInterval, 'ArrayValued', true) * B_cont
%            
%             % ensure symmetry
%             configOut.Q = (Q_d + Q_d') / 2;
%             configOut.R = (R_d + R_d') / 2;

            % enforce recomputation of controller, as A, B matrices changed
            Cache.clear()            
        case {'LinearPlant', 'NonlinearPlant'}
            plant = configIn.plant;
        otherwise
            error('ncs_initialize:InitPlant:UnsupportedPlantClass', ...
                '** Plant with name ''%s'' unsupported or unknown **',  configIn.plant);
    end
end

%% initSensor
function sensor = initSensor(config)
    if isfield(config, 'sensor') && isa(config.sensor, 'LinearMeasurementModel')
        % use the sensor/measurement model provided to create measurements
        % during the simulation
        sensor = config.sensor;
    else
        % use an ordinary linear measurement model and assume Gaussian noise
        assert(isfield(config, 'V'), ...
            'ncs_initialize:InitSensor:MeasNoiseCovMissing', ...
            '** Variable <V> must be present in the configuration to init a sensor producing noise-corrupted measurements **');
        sensor = LinearMeasurementModel(config.C);
        
        sensor.setNoise(Gaussian(getConfigValueOrDefault(config, 'v_mean', zeros(size(config.C, 1), 1)), config.V));        
    end
end

%% initController
function controller = initController(config, horizonLength)
    switch config.controllerClassName
        case 'InfiniteHorizonController'
            numCaModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS                
            transitionMatrixCa = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(config.caDelayProbs, numCaModes)); 
            
            controller = InfiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                transitionMatrixCa, config.controlSequenceLength);
            if controller.status == -1
                warning('ncs_initialize:InitController:InfiniteHorizonController', ...
                    ['** Warning: Numerical Problems: Gain of InfiniteHorizonController might ',...
                    'be corrupted. Working with best gain found. **']);
            elseif controller.status == 0
                warning('ncs_initialize:InitController:InfiniteHorizonController', ...
                    ['** Warning: InfiniteHorizonController diverged or convergence was not ',...
                    'detected. Working with best gain found. **']);
            end
        case 'FiniteHorizonController'
            numCaModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS                
            transitionMatrixCa = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(config.caDelayProbs, numCaModes)); 
            
            useMex = true; % by default, use the mex implementation to compute the gains
            if isfield(config, 'stateConstraintWeightings')
                assert(sum(isfield(config, {'inputConstraintWeightings', 'constraintBounds'})) == 2, ...
                    'ncs_initialize:InitController:FiniteHorizonController', ...
                    ['** Variables <inputConstraintWeightings> and <constraintBounds> ' ...
                    'must be present in the configuration to init the constrained FiniteHorizonController **']);
                
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    transitionMatrixCa, config.controlSequenceLength, horizonLength, useMex, ...
                    config.stateConstraintWeightings, config.inputConstraintWeightings, config.constraintBounds);
            else
                % no constraints in use, unconstrained control task
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    transitionMatrixCa, config.controlSequenceLength, horizonLength, useMex);
            end
        case 'LinearlyConstrainedPredictiveController'
            if sum(isfield(config, {'stateConstraintWeightings', 'stateConstraints'})) == 2
                stateConstraintWeightings = config.stateConstraintWeighting;
                stateConstraints = config.stateConstraints;
            else
                stateConstraintWeightings = [];
                stateConstraints = [];
            end
            if sum(isfield(config, {'inputConstraintWeightings', 'inputConstraints'})) == 2
                inputConstraintWeightings = config.stateConstraintWeighting;
                inputConstraints = config.inputConstraints;
            else
                inputConstraintWeightings = [];
                inputConstraints = [];
            end
            
            if sum(isfield(config, {'Z', 'refTrajectory'})) == 2
                % we track a trajectory
                controller = LinearlyConstrainedPredictiveController(config.A, config.B, config.Q, config.R, ...
                    config.controlSequenceLength, config.caDelayProbs, stateConstraintWeightings, stateConstraints, ...
                    inputConstraintWeightings, inputConstraints, config.Z, config.refTrajectory);
            else
                % we attempt to drive the state to the origin
                controller = LinearlyConstrainedPredictiveController(config.A, config.B, config.Q, config.R, ...
                    config.controlSequenceLength, config.caDelayProbs, stateConstraintWeightings, stateConstraints, ...
                    inputConstraintWeightings, inputConstraints);
            end
            if isfield(config, 'mpcHorizon')
                controller.changeHorizonLength(config.mpcHorizon);
            else
                warning('ncs_initialize:InitController:LinearlyConstrainedPredictiveController:NoHorizonLength', ...
                    'Horizon length K for %s not specified, using K=%d (sequence length) ***', ...
                    'LinearlyConstrainedPredictiveController', config.controlSequenceLength);                
            end
            
        case 'FiniteHorizonTrackingController'
            % check if the required additional fields are present in the
            % config: Z (matrix) and refTrajectory (matrix)
            assert(sum(isfield(config, {'Z', 'refTrajectory'})) == 2, ...
                'ncs_initialize:InitController:FiniteHorizonTrackingController', ...
                '** Variables <Z> and <refTrajectory> must be present in the configuration to init %s **', ...
                    'FiniteHorizonTrackingController');
            
            numCaModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS                
            transitionMatrixCa = Utility.calculateDelayTransitionMatrix(...
                Utility.truncateDiscreteProbabilityDistribution(config.caDelayProbs, numCaModes));
                
            controller = FiniteHorizonTrackingController(config.A, config.B, config.Q, config.R, config.Z, ...
                transitionMatrixCa, config.controlSequenceLength, horizonLength, config.refTrajectory);
        case 'EventTriggeredInfiniteHorizonController'
            assert(isfield(config, 'transmissionCosts'), ...
                'ncs_initialize:InitController:EventTriggeredInfiniteHorizonController', ...
                '** Variable <transmissionCosts> must be present in the configuration to init %s **', ...
                    'EventTriggeredInfiniteHorizonController');
            
            controller = EventTriggeredInfiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                config.caDelayProbs, config.controlSequenceLength, config.transmissionCosts);
        case 'NominalPredictiveController'
            controller = NominalPredictiveController(config.A, config.B, config.Q, config.R, ...
                config.controlSequenceLength);
        case 'ExpectedInputPredictiveController'
            controller = ExpectedInputPredictiveController(config.A, config.B, config.Q, config.R, ...
                config.controlSequenceLength, config.caDelayProbs);
        case 'ScenarioBasedPredictiveController'
              controller = ScenarioBasedPredictiveController(config.plant, config.Q, config.R, config.controlSequenceLength, ...
                  config.caDelayProbs);
              controller.changeHorizonLength(config.mpcHorizon);
        case 'PolePlacementPredictiveController'
            assert(isfield(config, 'polesCont'), ...
                'ncs_initialize:InitController:PolePlacementPredictiveController', ...
                '** Variable <polesCont> must be present in the configuration to init %s **', ...
                    'PolePlacementPredictiveController');
            controller = PolePlacementPredictiveController(config.A, config.B, config.polesCont, ...
                config.controlSequenceLength, config.samplingInterval);
        case 'InfiniteHorizonUdpLikeController'
            assert(isfield(config, 'scDelayProbs'), ...
                'ncs_initialize:InitController:InfiniteHorizonUdpLikeController', ...
                '** Variable <scDelayProbs> must be present in the configuration to init %s **', ...
                    'InfiniteHorizonUdpLikeController');
            
            assert(isfield(config, 'initialEstimate'), ...
                'ncs_initialize:InitController:InfiniteHorizonUdpLikeController', ...
                '** Variable <initialEstimate> must be present in the configuration to init %s **', ...
                    'InfiniteHorizonUdpLikeController');
            
            assert(isfield(config, 'V'), ...
                'ncs_initialize:InitController:InfiniteHorizonUdpLikeController', ...
                '** Variable <V> must be present in the configuration to init %s **', ...
                    'InfiniteHorizonUdpLikeController');
                
            if isfield(config, 'G')
                % transform the plant noise covariance
                actualW = config.G * config.W * config.G';
            else
                actualW = config.W;
            end            

            controller = InfiniteHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                config.caDelayProbs, config.scDelayProbs, config.controlSequenceLength, config.maxMeasDelay, ...
                actualW, config.V, getConfigValueOrDefault(config, 'v_mean', zeros(size(config.V, 1), 1)));

            controller.setControllerPlantState(config.initialEstimate);
        case 'IMMBasedRecedingHorizonController'
            assert(isfield(config, 'initialEstimate'), ...
                'ncs_initialize:InitController:IMMBasedRecedingHorizonController', ...
                '** Variable <initialEstimate> must be present in the configuration to init %s **', ...
                    'IMMBasedRecedingHorizonController');
            
            assert(isfield(config, 'V'), ...
                'ncs_initialize:InitController:IMMBasedRecedingHorizonController', ...
                '** Variable <V> must be present in the configuration to init %s **', ...
                    'IMMBasedRecedingHorizonController');
                
            if isfield(config, 'G')
                % transform the plant noise covariance
                actualW = config.G * config.W * config.G';
            else
                actualW = config.W;
            end
            
            measNoise = Gaussian(getConfigValueOrDefault(config, 'v_mean', zeros(size(config.V, 1), 1)), config.V);
            [x0, x0Cov] = config.initialEstimate.getMeanAndCov();
            if isfield(config, 'mpcHorizon')
                controllerHorizonLength = config.mpcHorizon;
            else
                warning('ncs_initialize:InitController:IMMBasedRecedingHorizonController:NoHorizonLength', ...
                    'Horizon length K for %s not specified, using K=%d (sequence length) ***', ...
                    'IMMBasedRecedingHorizonController', config.controlSequenceLength);
                controllerHorizonLength = config.controlSequenceLength;
            end
            numCaModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS    
            modeTransitionProbs = Utility.truncateDiscreteProbabilityDistribution(config.caDelayProbs, numCaModes);    
            transitionMatrixCa = Utility.calculateDelayTransitionMatrix(modeTransitionProbs); 
            
            controller = IMMBasedRecedingHorizonController(config.A, config.B, config.C, config.Q, config.R, ...
                    transitionMatrixCa, config.controlSequenceLength, config.maxMeasDelay, actualW, measNoise, ...
                    controllerHorizonLength, x0, x0Cov);
        case 'RecedingHorizonUdpLikeController'
            assert(isfield(config, 'initialEstimate'), ...
                'ncs_initialize:InitController:RecedingHorizonUdpLikeController', ...
                '** Variable <initialEstimate> must be present in the configuration to init %s **', ...
                    'RecedingHorizonUdpLikeController');
            assert(isfield(config, 'scDelayProbs'), ...
                'ncs_initialize:InitController:RecedingHorizonUdpLikeController', ...
                '** Variable <scDelayProbs> must be present in the configuration to init %s **', ...
                    'RecedingHorizonUdpLikeController');
            assert(isfield(config, 'V'), ...
                'ncs_initialize:InitController:RecedingHorizonUdpLikeController', ...
                '** Variable <V> must be present in the configuration to init %s **', ...
                    'RecedingHorizonUdpLikeController');
               
            if isfield(config, 'G')
                % transform the plant noise covariance
                actualW = config.G * config.W * config.G';
            else
                actualW = config.W;
            end
            scDelayProbs = config.scDelayProbs;            
            [x0, x0Cov] = config.initialEstimate.getMeanAndCov();
            if isfield(config, 'mpcHorizon')
                controllerHorizonLength = config.mpcHorizon;
            else
                warning('ncs_initialize:InitController:RecedingHorizonUdpLikeController:NoHorizonLength', ...
                    'Horizon length K for %s not specified, using K=%d (sequence length) ***', ...
                    'RecedingHorizonUdpLikeController', config.controlSequenceLength);
                controllerHorizonLength = config.controlSequenceLength;
            end
            numCaModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS    
            modeTransitionProbs = Utility.truncateDiscreteProbabilityDistribution(config.caDelayProbs, numCaModes);    
            transitionMatrixCa = Utility.calculateDelayTransitionMatrix(modeTransitionProbs); 
            
            controller = RecedingHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                transitionMatrixCa, scDelayProbs, config.controlSequenceLength, ...
                config.maxMeasDelay, actualW, config.V, controllerHorizonLength, x0, x0Cov, true);
        case 'MSSController'
            controllerDelta = 0.1; % to be promoted to configuration parameter
            %controller = MSSController(config.A, config.B, config.controlSequenceLength, controllerDelta);
            [x0, ~] = config.initialEstimate.getMeanAndCov();
            controller = DynamicMSSController(config.A, config.B, config.C, config.controlSequenceLength, ...
                controllerDelta, config.maxMeasDelay, x0);
        otherwise
            error('ncs_initialize:InitController:UnsupportedControllerClass', ...
                '** Controller class with name ''%s'' unsupported or unknown **', config.controllerClassName);
    end
end

%% initFilter
function [filter, filterPlantModel, filterSensorModel] = initFilter(config)
    assert(isfield(config, 'initialEstimate'), ...
        'ncs_initialize:InitFilter:InitialEstimateMissing', ...
        '** Variable <initialEstimate> must be present in the configuration to init a filter/estimator **');
    assert(isfield(config, 'V'), ...
        'ncs_initialize:InitFilter:MeasNoiseCovMissing', ...
        '** Variable <V> must be present in the configuration to init a filter/estimator **');
    
    caDelayProbs = config.caDelayProbs;
    Validator.validateDiscreteProbabilityDistribution(caDelayProbs);    
   
    numModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS    
    modeTransitionProbs = Utility.truncateDiscreteProbabilityDistribution(caDelayProbs, numModes);    
    
    transitionMatrix = Utility.calculateDelayTransitionMatrix(modeTransitionProbs); 
    filterPlantModel = [];
    filterSensorModel = [];
    switch config.filterClassName
        case 'DelayedKF'
            delayWeights = Utility.computeStationaryDistribution(transitionMatrix);
            filter = DelayedKF(config.maxMeasDelay, transitionMatrix);
            
            filterPlantModel = DelayedKFSystemModel(config.A, ...
                config.B, Gaussian(zeros(size(config.A, 1), 1), config.W), ...
                numModes, config.maxMeasDelay, delayWeights);
            if isfield(config, 'G')
                filterPlantModel.setSystemNoiseMatrix(config.G);
            end 
        case 'DelayedModeIMMF'
            modeFilters = arrayfun(@(mode) EKF(sprintf('KF for mode %d', mode)), 1:numModes, 'UniformOutput', false);
            filter = DelayedModeIMMF(modeFilters, transitionMatrix, config.maxMeasDelay);
             
            filterPlantModel = JumpLinearSystemModel(numModes, ...
                arrayfun(@(~) LinearPlant(config.A, config.B, config.W), ...
                    1:numModes, 'UniformOutput', false));
            if isfield(config, 'G')
                % add the G matrix to all mode-conditioned models
                cellfun(@(model) model.setSystemNoiseMatrix(config.G), filterPlantModel.modeSystemModels);
            end
        otherwise
            error('ncs_initialize:InitFilter:UnsupportedFilterClass', ...
                '** Filter class with name ''%s'' unsupported or unknown **', config.filterClassName);
    end
    % use linear measurement model
    filterSensorModel = LinearMeasurementModel(config.C);
    fields = isfield(config, {'v_mean', 'V'});
    if fields(1)
        noiseMean = config.v_mean;
    else
        noiseMean = zeros(size(config.C, 1), 1);
    end
    if fields(2)
        % noise cov was given, so assume a Gaussion noise
        filterSensorModel.setNoise(Gaussian(noiseMean, config.V))
    else
        % noise is a mere static offset
        % probabilistically modeled in the form of a Dirac mixture
        % with a single component
        filterSensorModel.setNoise(DiracMixture(noiseMean, 1));
    end
    filter.setState(config.initialEstimate);
end

%% writeCacheInfo
function writeCacheInfo(configFileName, configFilePath, cacheInfo)
    cacheInfoFile = [configFilePath filesep configFileName '.cacheinfo'];
    [fileId, errMsg] = fopen(cacheInfoFile, 'w', 'n', 'UTF-8');
    assert(fileId > 0, 'ncs_initialize:WriteCacheInfo:Fopen', errMsg);
    
    % save date as full precision double, name as string, and size as integer
    fprintf(fileId, 'name %s\ndate %.15f\nsize %d', cacheInfo.name, cacheInfo.date, cacheInfo.size);
    fclose(fileId);
end

%% readCacheInfo
function cacheInfo = readCacheInfo(configFileName, configFilePath)
    % first check if cacheinfo file is present
    cacheInfo = [];
    cacheInfoFile = [configFilePath filesep configFileName '.cacheinfo'];
    if exist(cacheInfoFile, 'file') == 2
        [fileId, errMsg] = fopen(cacheInfoFile, 'r', 'n', 'UTF-8');
        
        assert(fileId > 0, 'ncs_initialize:ReadCacheInfo:Fopen', errMsg);

        % create the structure from the file
        cacheInfo.name = sscanf(fgetl(fileId), 'name %s');
        cacheInfo.date = sscanf(fgetl(fileId), 'date %f');
        cacheInfo.size = sscanf(fgetl(fileId), 'size %d');
        fclose(fileId);
    end
end

%% checkConfig
function checkConfig(config, expectedVariables)
     %configVars = who('-file', configFile);
     %found = ismember(expectedVariables, configVars);
     % config is a struct, so look for fieldnames
     found = isfield(config, expectedVariables);
     notFoundIdx = find(~found);
     assert(numel(notFoundIdx) == 0, ...
        'ncs_initialize:CheckConfigFile', ...
         '** The following %d variables must be present either in the config struct or the provided config file: %s **', ...
         numel(notFoundIdx), strjoin(expectedVariables(notFoundIdx), ','));     
end

%% getConfigValueOrDefault
function value = getConfigValueOrDefault(config, fieldname, defaultValue)
    if isfield(config, fieldname)
        value = config.(fieldname);        
    else
        value = defaultValue;
    end
end
