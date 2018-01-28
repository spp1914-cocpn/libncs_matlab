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
        'caDelayProbs', 'scDelayProbs', 'A', 'B', 'C', 'W', 'V', 'Q', 'R', ...
        'initialPlantState', 'initialEstimate', 'samplingInterval', 'controllerClassName'};
    % presence of a filter class is not required, as some controllers can
    % do this internally % 'filterClassName'
    
    if ~isempty(configStruct) && ~(isstruct(configStruct) && isscalar(configStruct))
        error('ncs_initialize:InvalidConfigStruct', ...
            '** <configStruct> must be a single struct **');
    end
    % create NCS and initialize
    % return unique handle
    if ~ischar(filename) || min(size(filename)) ~= 1
        error('ncs_initialize:InvalidFilename', ...
            '** <filename> must be a character vector **');
    end
    [pathStr, configFileName, extension] = fileparts(filename);
    if exist(filename, 'file') ~= 2 || ~strcmp(extension, '.mat') % 2 is returned in case of existing file
        error('ncs_initialize:InvalidFile', ...
            '** %s does not exist or is not a mat-file **', filename);
    end
   
    if ~Checks.isPosScalar(maxSimTime)
        error('ncs_initialize:InvalidMaxSimeTime', ...
            '** <maxSimTime> must be positive and given in pico-seconds. **');
    end
    
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
     
    ncs = NetworkedControlSystem(id, config.samplingInterval);
    loopSteps = floor(ConvertToSeconds(maxSimTime) / ncs.samplingInterval);
      
    % try to reuse controller, if possible
    controller = doControllerCacheLookup(config, configFileName, pathStr, loopSteps);
    if isempty(controller)
        controller = initController(config, loopSteps);
        newCacheInfo = Cache.insert(controller, [configFileName '_controller_' int2str(loopSteps)]);
        writeCacheInfo(configFileName, pathStr, newCacheInfo);
    end
    
    plant = LinearPlant(config.A, config.B, config.W);
    % there might be an addtional system noise matrix present (G)
    % if not, default G = I is used
    if isfield(config, 'G')
        plant.setSystemNoiseMatrix(config.G);    
    end
    actuator = initActuator(config);
    ncs.plant = NcsPlant(plant, actuator);
        
    measModel = LinearMeasurementModel(config.C);
    measModel.setNoise(Gaussian(zeros(size(config.C, 1), 1), config.V))
    ncs.sensor = NcsSensor(measModel);
    
    if isfield(config, 'filterClassName')
        % we need a filter
        [filter, augmentedPlantModel] = initFilter(config, plant);
        if ~isempty(augmentedPlantModel)
            filterSpecificPlantModel = augmentedPlantModel;
        else
            filterSpecificPlantModel = plant;
        end
        ncs.controller = NcsControllerWithFilter(controller, filter, ...
            filterSpecificPlantModel, measModel, actuator.defaultInput);
    else
        % controller does not require an external filter
        ncs.controller = NcsController(controller);
    end
    ncs.initPlant(config.initialPlantState);
    ncs.initStatisticsRecording(loopSteps);
    
    handle = ComponentMap.getInstance().addComponent(ncs);
    clear('config');
end

%% doControllerCacheLookup
function controller = doControllerCacheLookup(config, configFileName, pathStr, loopSteps)
    cacheInfo = readCacheInfo(configFileName, pathStr);
    controller = [];
    if ~isempty(cacheInfo)
        cachedController = Cache.lookup(cacheInfo);
        if ~isempty(cachedController) && strcmp(class(cachedController), config.controllerClassName)
            % check for additional, type dependent proprties that must
            % match
            switch config.controllerClassName
                case 'FiniteHorizonController'
                    if cachedController.horizonLength == loopSteps ...
                            && cachedController.sequenceLength == config.controlSequenceLength
                        controller = cachedController;
                    end
                case 'FiniteHorizonTrackingController'
                    if cachedController.horizonLength == loopSteps ...
                            && cachedController.sequenceLength == config.controlSequenceLength ...
                            && isequal(config.refTrajectory, controller.refTrajectory)
                        controller = cachedController;
                    end
                case 'NominalPredictiveController'
                    if cachedController.sequenceLength ~= config.controlSequenceLength
                        cachedController.changeSequenceLength(config.controlSequenceLength);
                    end
                    controller = cachedController;
                case 'InfiniteHorizonController'
                    if cachedController.sequenceLength == config.controlSequenceLength
                        controller = cachedController;
                    end
                case 'InfiniteHorizonUdpLikeController'
                    if cachedController.sequenceLength == config.controlSequenceLength ...
                            && cachedController.maxMeasurementDelay == config.maxMeasDelay
                        controller = cachedController;
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
            if isfield(config, 'stateConstraintWeightings')
                if sum(isfield(config, {'inputConstraintWeightings', 'constraintBounds'})) ~= 2
                    error('ncs_initialize:InitController:FiniteHorizonController', ...
                        ['** Variables <inputConstraintWeightings> and <constraintBounds> ' ...
                        'must be present in the configuration to init the constrained FiniteHorizonController **']);
                end
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    config.caDelayProbs, config.controlSequenceLength, horizonLength, ...
                    config.stateConstraintWeightings, config.inputConstraintWeightings, config.constraintBounds);
            else
                % no constraints in use, unconstrained control task
                controller = FiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                    config.caDelayProbs, config.controlSequenceLength, horizonLength);
            end
        case 'FiniteHorizonTrackingController'
            % check if the required additional fields are present in the
            % config: Z (matrix) and refTrajectory (matrix)
            if sum(isfield(config, {'Z', 'refTrajectory'})) ~= 2
                error('ncs_initialize:InitController:FiniteHorizonTrackingController', ...
                    '** Variables <Z> and <refTrajectory> must be present in the configuration to init %s**', ...
                    'FiniteHorizonTrackingController');
            end
            controller = FiniteHorizonTrackingController(config.A, config.B, config.Q, config.R, config.Z, ...
                config.caDelayProbs, config.controlSequenceLength, horizonLength, config.refTrajectory);
        case 'EventTriggeredInfiniteHorizonController'
            if ~isfield(config, 'transmissionCosts')
                error('ncs_initialize:InitController:EventTriggeredInfiniteHorizonController', ...
                    '** Variable <transmissionCosts> must be present in the configuration to init %s **', ...
                    'EventTriggeredInfiniteHorizonController');
            end
            controller = EventTriggeredInfiniteHorizonController(config.A, config.B, config.Q, config.R, ...
                config.caDelayProbs, config.controlSequenceLength, config.transmissionCosts);
        case 'NominalPredictiveController'
            controller = NominalPredictiveController(config.A, config.B, config.Q, config.R, ...
                config.controlSequenceLength);
        case 'InfiniteHorizonUdpLikeController'
            controller = InfiniteHorizonUdpLikeController(config.A, config.B, config.C, config.Q, config.R, ...
                config.caDelayProbs, config.scDelayProbs, config.controlSequenceLength, config.maxMeasDelay, ...
                config.W, config.V);
        otherwise
            error('ncs_initialize:InitController:UnsupportedControllerClass', ...
                '** Controller class with name ''%s'' unsupported or unknown **', config.controllerClassName);
    end
end

%% initFilter
function [filter, augmentedPlantModel] = initFilter(config, plant)
    caDelayProbs = config.caDelayProbs;
    Validator.validateDiscreteProbabilityDistribution(caDelayProbs);
    
    numProbs = length(caDelayProbs);
    numModes = config.controlSequenceLength + 1; % number of modes of the augmented plant MJLS
    
    if numProbs < numModes
        % fill up with zeros
        modeTransitionProbs = [caDelayProbs(:); zeros(numModes - numProbs, 1)];
    elseif numModes < numProbs
        % cut the distribution and fill up with an entry so that
        % sum is 1 again
        modeTransitionProbs = [caDelayProbs(1:1:numModes -1), 1 - sum(caDelayProbs(1:1:numModes -1))];
    else
        modeTransitionProbs = caDelayProbs(:);
    end
    transitionMatrix = Utility.calculateDelayTransitionMatrix(modeTransitionProbs); 
    
    switch config.filterClassName
        case 'DelayedKF'
            delayWeights = Utility.computeStationaryDistribution(transitionMatrix);
            filter = DelayedKF(config.maxMeasDelay);
            augmentedPlantModel = DelayedKFSystemModel(plant.sysMatrix, ...
                plant.inputMatrix, plant.noise, numModes, config.maxMeasDelay, delayWeights);
        case 'DelayedModeIMMF'
            modeFilters = FilterSet();
            arrayfun(@(mode) modeFilters.add(AnalyticKF(sprintf('KF for mode %d', mode))), 1:numModes);
            filter = DelayedModeIMMF(modeFilters, transitionMatrix, config.maxMeasDelay);
            [~, plantNoiseCov] = plant.noise.getMeanAndCovariance();
            augmentedPlantModel = JumpLinearSystemModel(numModes, ...
                arrayfun(@(~) LinearPlant(plant.sysMatrix, plant.inputMatrix, plantNoiseCov), ...
                    1:numModes, 'UniformOutput', false));
        case 'DelayedIMMF'
            modeFilters = FilterSet();
            arrayfun(@(mode) modeFilters.add(AnalyticKF(sprintf('KF for mode %d', mode))), 1:numModes);
            filter = DelayedIMMF(modeFilters, transitionMatrix, config.maxMeasDelay);
            [~, plantNoiseCov] = plant.noise.getMeanAndCovariance();
            augmentedPlantModel = JumpLinearSystemModel(numModes, ...
                arrayfun(@(~) LinearPlant(plant.sysMatrix, plant.inputMatrix, plantNoiseCov), ...
                    1:numModes, 'UniformOutput', false));
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
    if fileId > 0
        % save date as full precision double, name as string, and size as
        % integer
        fprintf(fileId, 'name %s\ndate %.15f\nsize %d', cacheInfo.name, cacheInfo.date, cacheInfo.size);
        fclose(fileId);
    else
        error('ncs_initialize:WriteCacheInfo:Fopen', errMsg);
    end
end

%% readCacheInfo
function cacheInfo = readCacheInfo(configFileName, configFilePath)
    % first check if cacheinfo file is present
    cacheInfo = [];
    cacheInfoFile = [configFilePath filesep configFileName '.cacheinfo'];
    if exist(cacheInfoFile, 'file') == 2
        [fileId, errMsg] = fopen(cacheInfoFile, 'r', 'n', 'UTF-8');
        if fileId > 0
            % create the structure from the file
            cacheInfo.name = sscanf(fgetl(fileId), 'name %s');
            cacheInfo.date = sscanf(fgetl(fileId), 'date %f');
            cacheInfo.size = sscanf(fgetl(fileId), 'size %d');
            fclose(fileId);
        else
            error('ncs_initialize:ReadCacheInfo:Fopen', errMsg);
        end
    end
end

%% checkConfig
function checkConfig(config, expectedVariables)
     %configVars = who('-file', configFile);
     %found = ismember(expectedVariables, configVars);
     % config is a struct, so look for fieldnames
     found = isfield(config, expectedVariables);
     notFoundIdx = find(~found);
     if numel(notFoundIdx) > 0
         error('ncs_initialize:checkConfigFile', ...
             '** The following %d variables must be present either in the config struct or the provided config file: %s **', ...
             numel(notFoundIdx), strjoin(expectedVariables(notFoundIdx), ','));
     end
end

