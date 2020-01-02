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
    requiredVariables = {'controlSequenceLength', 'maxControlSequenceDelay', 'maxMeasDelay', ...
        'caDelayProbs', 'A', 'B', 'C', 'W', 'V', 'Q', 'R', ...
        'initialPlantState', 'samplingInterval', 'controllerClassName'};
    % optional variables:
    %   - scDelayProbs: specifies the true or assumed distribution of the delays in the
    %     network between sensor and controller (i.e., for measurements);
    %     required by some controllers or filters (nonnegative vector) 
    %   - filterClassName: if a filter is required for state estimation
    %   - initialEstimate: the initial controller state or the initial
    %     estimate of the employed filter (if any)
    %   - networkType: specifies the network in use (NetworkType)
    %   - Z: performance output matrix for the plant output z_k = Z * x_k
    %   - refTrajectory: reference trajectory z_ref to track with the plant output z_k
    %   - v_mean: mean of the measurement noise (default zero)
    %   - plant: arbitrary, nonlinear plant dynamics to be used for simulation (NonlinearPlant)
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
    %   - stateConstraintWeightings: linear constraints weightings c_i for MPC, i.e., c_i'*u_k <= d_i
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
        % combine data from config file and from struct into a single config
        % struct
        config = configStruct;
        tmp = load(filename);
        fields = fieldnames(tmp);
        for i=1:length(fields)
            name = fields{i};
            if ~isfield(config, name)
                % append config value from the mat file
                config.(name) = tmp.(name);
            end
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

    plant = initPlant(config);
    actuator = initActuator(config);
    ncsPlant = NcsPlant(plant, actuator);
    
    if isfield(config, 'ignoreControllerCache') && ~config.ignoreControllerCache
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
    if isfield(config, 'sensorEventBased') && config.sensorEventBased
        ncsSensor = EventBasedNcsSensor(sensor);
        if isfield(config, 'sensorMeasDelta')
            ncsSensor.measurementDelta = config.sensorMeasDelta;
        end
    else
        ncsSensor = NcsSensor(sensor);
    end
    
    if controller.requiresExternalStateEstimate
        % we need a filter
        assert(isfield(config, 'filterClassName'), ...
            'ncs_initialize:FilterClassNameMissing', ...
            ['** Variable <filterClassName> must be present in the configuration to init a filter/estimator '...
            'required by %s **'], class(controller));
        ncsController = initNcsControllerWithFilter(config, controller, sensor, actuator);
    else
        % controller does not require an external filter
        ncsController = initNcsController(config, controller, actuator);
    end    
    
    if isfield(config, 'controlErrorWindowSize')
        ncsController.controlErrorWindowSize = config.controlErrorWindowSize;
    end
    
    % init NCS: check if network type was specified
    if isfield(config, 'networkType')
        ncs = NetworkedControlSystem(ncsController, ncsPlant, ncsSensor, id, config.samplingInterval, config.networkType);
    else
        ncs = NetworkedControlSystem(ncsController, ncsPlant, ncsSensor, id, config.samplingInterval);
    end
    
    ncs.initPlant(config.initialPlantState);
    ncs.initStatisticsRecording(loopSteps);
    
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
function ncsControllerWithFilter = initNcsControllerWithFilter(config, controller, sensor, actuator)
    % we need a filter
    [filter, augmentedPlantModel, sensorModel] = initFilter(config);
    if ~isempty(augmentedPlantModel)
        filterSpecificPlantModel = augmentedPlantModel;
    else
        filterSpecificPlantModel = plant;
    end
    if ~isempty(sensorModel)
        filterSpecificSensorModel = sensorModel;
    else
        filterSpecificSensorModel = sensor;
    end
    if isfield(config, 'controllerEventBased') && config.controllerEventBased
        if isfield(config, 'linearizationPoint')
            ncsControllerWithFilter = EventBasedNcsControllerWithFilter(controller, filter, ...
            filterSpecificPlantModel, filterSpecificSensorModel, actuator.defaultInput, config.caDelayProbs, config.linearizationPoint);
        else
            ncsControllerWithFilter = EventBasedNcsControllerWithFilter(controller, filter, ...
            filterSpecificPlantModel, filterSpecificSensorModel, actuator.defaultInput, config.caDelayProbs);
        end
        % set the deadband, if provided (else default value is used)
        if isfield(config, 'controllerDeadband')
           ncsControllerWithFilter.deadband = config.controllerDeadband;
        end
        % set the event trigger, if provided (else default value is used)
        if isfield(config, 'controllerEventTrigger')
            ncsControllerWithFilter.eventTrigger = uint8(config.controllerEventTrigger);
        end
    elseif isfield(config, 'linearizationPoint')
        ncsControllerWithFilter = NcsControllerWithFilter(controller, filter, ...
            filterSpecificPlantModel, filterSpecificSensorModel, actuator.defaultInput, config.caDelayProbs, config.linearizationPoint);
    else
        ncsControllerWithFilter = NcsControllerWithFilter(controller, filter, ...
            filterSpecificPlantModel, filterSpecificSensorModel, actuator.defaultInput, config.caDelayProbs);
    end
end

%% initNcsController
function ncsController = initNcsController(config, controller, actuator)
    % controller does not require an external filter
     if isfield(config, 'controllerEventBased') && config.controllerEventBased
         % we use an event-based controller with deadband strategy
        if isfield(config, 'linearizationPoint')
            ncsController = EventBasedNcsController(controller, actuator.defaultInput, config.linearizationPoint);
         else
             ncsController = EventBasedNcsController(controller, actuator.defaultInput);
         end
         % set the deadband, if provided (else default value is used)
        if isfield(config, 'controllerDeadband')
           ncsController.deadband = config.controllerDeadband;
        end
         % set the event trigger, if provided (else default value is used)
        if isfield(config, 'controllerEventTrigger')
            ncsController.eventTrigger = uint8(config.controllerEventTrigger);
        end
     elseif isfield(config, 'linearizationPoint')
        ncsController = NcsController(controller, actuator.defaultInput, config.linearizationPoint);
     else
         ncsController = NcsController(controller, actuator.defaultInput);
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
    actuator = BufferingActuator(config.controlSequenceLength, ...
        config.maxControlSequenceDelay, zeros(size(config.B,2), 1));
end

%% initPlant
function plant = initPlant(config)
    if isfield(config, 'plant') && isa(config.plant, 'NonlinearPlant')
        plant = config.plant;
        if isa(config.plant, 'InvertedPendulum') && config.samplingInterval ~= plant.samplingInterval
            
            plant.samplingInterval = config.samplingInterval;
            [config.A, config.B, config.C, config.W] = plant.linearizeAroundUpwardEquilibrium(config.W_cont);
                     
            % use discrete-time noise also for the nonlinear plant dynamics
            plant.setNoise(Gaussian(zeros(4, 1), config.W));
            
            % enforce recomputation of controller, as A, B matrices changed
            Cache.clear()
        end
    else
        % use an ordinary linear plant
        plant = LinearPlant(config.A, config.B, config.W);
        % there might be an additional system noise matrix present (G)
        % if not, default G = I is used
        if isfield(config, 'G')
            plant.setSystemNoiseMatrix(config.G);    
        end
    end
end

%% initSensor
function sensor = initSensor(config)
    if isfield(config, 'sensor') && isa(config.sensor, 'LinearMeasurementModel')
        sensor = config.sensor;
    else
        % use an ordinary linear measurement model and assume Gaussian noise
        sensor = LinearMeasurementModel(config.C);
        if isfield(config, 'v_mean')
            noiseMean = config.v_mean;
        else
            noiseMean = zeros(size(config.C, 1), 1);
        end
    sensor.setNoise(Gaussian(noiseMean, config.V))
    end
end

%% initController
function controller = initController(config, horizonLength)
    switch config.controllerClassName
        case 'InfiniteHorizonController'
            controller = InfiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                config.caDelayProbs, config.controlSequenceLength);
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
            useMex = true; % by default, use the mex implementation to compute the gains
            if isfield(config, 'stateConstraintWeightings')
                assert(sum(isfield(config, {'inputConstraintWeightings', 'constraintBounds'})) == 2, ...
                    'ncs_initialize:InitController:FiniteHorizonController', ...
                    ['** Variables <inputConstraintWeightings> and <constraintBounds> ' ...
                    'must be present in the configuration to init the constrained FiniteHorizonController **']);
                
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    config.caDelayProbs, config.controlSequenceLength, horizonLength, useMex, ...
                    config.stateConstraintWeightings, config.inputConstraintWeightings, config.constraintBounds);
            else
                % no constraints in use, unconstrained control task
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    config.caDelayProbs, config.controlSequenceLength, horizonLength, useMex);
            end
        case 'LinearlyConstrainedPredictiveController'
            assert(sum(isfield(config, {'stateConstraintWeightings', 'inputConstraintWeightings', ...
                    'stateConstraints', 'inputConstraints'})) == 4, ...
                'ncs_initialize:InitController:LinearlyConstrainedPredictiveController', ...
                ['** Variables <stateConstraintWeightings>, <inputConstraintWeightings>, ' ...
                '<stateConstraints> and <inputConstraints> '...
                'must be present in the configuration to init LinearlyConstrainedPredictiveController **']);
            
            if sum(isfield(config, {'Z', 'refTrajectory'})) == 2
                % we track a trajectory
                controller = LinearlyConstrainedPredictiveController(config.A, config.B, config.Q, config.R, ...
                    config.controlSequenceLength, config.stateConstraintWeightings, config.stateConstraints, ...
                    config.inputConstraintWeightings, config.inputConstraints, config.Z, config.refTrajectory);
            else
                % we attempt to drive the state to the origin
                controller = LinearlyConstrainedPredictiveController(config.A, config.B, config.Q, config.R, ...
                    config.controlSequenceLength, config.stateConstraintWeightings, config.stateConstraints, ...
                    config.inputConstraintWeightings, config.inputConstraints);
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
            
            controller = FiniteHorizonTrackingController(config.A, config.B, config.Q, config.R, config.Z, ...
                config.caDelayProbs, config.controlSequenceLength, horizonLength, config.refTrajectory);
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
        case 'InfiniteHorizonUdpLikeController'
            assert(isfield(config, 'scDelayProbs'), ...
                'ncs_initialize:InitController:InfiniteHorizonUdpLikeController', ...
                '** Variable <scDelayProbs> must be present in the configuration to init %s **', ...
                    'InfiniteHorizonUdpLikeController');
            
            assert(isfield(config, 'initialEstimate'), ...
                'ncs_initialize:InitController:InfiniteHorizonUdpLikeController', ...
                '** Variable <initialEstimate> must be present in the configuration to init %s **', ...
                    'InfiniteHorizonUdpLikeController');
                
            if isfield(config, 'G')
                % transform the plant noise covariance
                actualW = config.G * config.W * config.G';
            else
                actualW = config.W;
            end            
            if isfield(config, 'v_mean')
                controller = InfiniteHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                    config.caDelayProbs, config.scDelayProbs, config.controlSequenceLength, config.maxMeasDelay, ...
                    actualW, config.V, config.v_mean);
            else
                controller = InfiniteHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                    config.caDelayProbs, config.scDelayProbs, config.controlSequenceLength, config.maxMeasDelay, ...
                    actualW, config.V);
            end
            controller.setControllerPlantState(config.initialEstimate);
        case 'IMMBasedRecedingHorizonController'
            assert(isfield(config, 'initialEstimate'), ...
                'ncs_initialize:InitController:IMMBasedRecedingHorizonController', ...
                '** Variable <initialEstimate> must be present in the configuration to init %s **', ...
                    'IMMBasedRecedingHorizonController');
            
            if isfield(config, 'G')
                % transform the plant noise covariance
                actualW = config.G * config.W * config.G';
            else
                actualW = config.W;
            end
            if isfield(config, 'v_mean')
                v_mean = config.v_mean;
            else
                v_mean = zeros(size(config.C, 1), 1);
            end
            measNoise = Gaussian(v_mean, config.V);
            [x0, x0Cov] = config.initialEstimate.getMeanAndCov();
            if isfield(config, 'mpcHorizon')
                controllerHorizonLength = config.mpcHorizon;
            else
                warning('ncs_initialize:InitController:IMMBasedRecedingHorizonController:NoHorizonLength', ...
                    'Horizon length K for %s not specified, using K=%d (sequence length) ***', ...
                    'IMMBasedRecedingHorizonController', config.controlSequenceLength);
                controllerHorizonLength = config.controlSequenceLength;
            end
            controller = IMMBasedRecedingHorizonController(config.A, config.B, config.C, config.Q, config.R, ...
                    config.caDelayProbs, config.controlSequenceLength, config.maxMeasDelay, actualW, measNoise, ...
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
            controller = RecedingHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                config.caDelayProbs, scDelayProbs, config.controlSequenceLength, ...
                config.maxMeasDelay, actualW, config.V, controllerHorizonLength, x0, x0Cov);
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
            filter = DelayedKF(config.maxMeasDelay);
            
            filterPlantModel = DelayedKFSystemModel(config.A, ...
                config.B, Gaussian(zeros(size(config.A, 1), 1), config.W), ...
                numModes, config.maxMeasDelay, delayWeights);
            if isfield(config, 'G')
                filterPlantModel.setSystemNoiseMatrix(config.G);
            end
            % use linear measurement model and assume Gaussian noise
            filterSensorModel = LinearMeasurementModel(config.C);
            if isfield(config, 'v_mean')
                noiseMean = config.v_mean;
            else
                noiseMean = zeros(size(config.C, 1), 1);
            end
            filterSensorModel.setNoise(Gaussian(noiseMean, config.V))
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
             % use linear measurement model and assume Gaussian noise
            filterSensorModel = LinearMeasurementModel(config.C);
            if isfield(config, 'v_mean')
                noiseMean = config.v_mean;
            else
                noiseMean = zeros(size(config.C, 1), 1);
            end
            filterSensorModel.setNoise(Gaussian(noiseMean, config.V))
        case 'DelayedIMMF'
            modeFilters = arrayfun(@(mode) EKF(sprintf('KF for mode %d', mode)), 1:numModes, 'UniformOutput', false);
            filter = DelayedIMMF(modeFilters, transitionMatrix, config.maxMeasDelay);

            filterPlantModel = JumpLinearSystemModel(numModes, ...
                arrayfun(@(~) LinearPlant(config.A, config.B, config.W), ...
                    1:numModes, 'UniformOutput', false));
            if isfield(config, 'G')
                % add the G matrix to all mode-conditioned models
                cellfun(@(model) model.setSystemNoiseMatrix(config.G), filterPlantModel.modeSystemModels);
            end
            % use linear measurement model and assume Gaussian noise
            filterSensorModel = LinearMeasurementModel(config.C);
            if isfield(config, 'v_mean')
                noiseMean = config.v_mean;
            else
                noiseMean = zeros(size(config.C, 1), 1);
            end
            filterSensorModel.setNoise(Gaussian(noiseMean, config.V))
        otherwise
            error('ncs_initialize:InitFilter:UnsupportedFilterClass', ...
                '** Filter class with name ''%s'' unsupported or unknown **', config.filterClassName);
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

