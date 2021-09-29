function [pktsOut, qocOut, stats, controllerStateAdmissible] = ncs_doLoopStep(handle, timestamp, paramStruct)
    % Perform a control loop (NCS) cycle in Matlab. 
    %
    % Parameters:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem.
    %
    %   >> timestamp (Positive Integer)
    %      The current simulation time (in Omnet), in pico-seconds.
    %
    %   >> paramStruct (Struct or empty matrix)
    %      A structure containing parameters of the NCS to be changed or newly set 
    %      at the beginning of this control cycle.
    %      A typical example would be an adaptation of the controller
    %      deadband.
    %      Supported parameters to date:
    %      - controllerDeadband (Nonnegative scalar): the new threshold value (deadband) for the decison rule
    %        if the controller employs a deadband control strategy; ignored, if controller is not
    %        event-based or does not employ a deadband control strategy
    %      - sensorMeasDelta (Nonnegative scalar): the new threshold value (delta) for the decision rule
    %        if the sensor employs a send-on-delta strategy; 
    %        ignored, if the sensor is not event-based
    %      - controlSequenceLength (Positive integer): the new length of the control sequences
    %        to be transmitted by the controller from now on;
    %        ignored, if changing the sequence length at runtime is not
    %        supported by the employed controller
    %      - caDelayProbs (Nonnegative vector): the new true (or assumed) distribution of the delays in the
    %        network between controller and actuator (i.e., for control sequences);
    %        ignored, if adapting this distribution at runtime is not
    %        supported by the employed controller
    %      - scDelayProbs (Nonnegative vector): the new true (or assumed) distribution of the delays in the
    %        network between sensor and controller (i.e., for measurements);
    %        ignored, if adapting this distribution at runtime is not
    %        supported by the employed controller
    %
    % Returns:
    %   << pktsOut (Cell array, column-vector like, of DataPackets)
    %      The DataPackets resulting from the execution of the cycle to be
    %      sent (e.g., measurements, control sequence), row-wise arranged
    %      in a cell array.
    %
    %   << qocOut (Nonnegative scalar)
    %      The current Quality of Control (QoC), as perceived by the
    %      controller, which is reported to the communication system an
    %      used for congestion control computations.
    %
    %   << stats (Struct)
    %      Struct containing statistical data gathered during the execution
    %      of the control cycle. 
    %      At least the following fields are present:
    %      - actual_control_error (Nonnegative scalar), indicating the
    %        current true value of the control error measure
    %      - estimated_control_error (Nonnegative scalar), indicating the value of the error measure as perceived/estimated by the controller
    %      - actual_stagecosts (Nonnegative scalar), indicating the current
    %        stage costs according the controller's underlying cost functional
    %      - sc_delays (Column vector of nonnegative integers, might be empty), 
    %        describing the delays (in time steps) the processed DataPackets sent from the sensor experienced
    %      - ca_delays (Column vector of nonnegative integers, might be empty), 
    %        describing the delays (in time steps) the processed DataPackets sent from the controller experienced
    %      - ac_delays (Column vector of nonnegative integers, might be empty), 
    %        describing the delays (in time steps) the processed ACKs sent from the actuator experienced
    %      - sc_sent (Logical, i.e., a flag), indicating whether a
    %        packet (i.e., a measurement) was sent out by the sensor or not
    %      - ca_sent (Logical, i.e., a flag), indicating whether a
    %        packet (i.e., a control sequence) was sent out by the controller or not
    %      - ac_sent (Logical, i.e., a flag), indicating whether a
    %        packet (i.e., an ACK) was sent out by the actuator or not
    %
    %   << controllerStateAdmissible (Logical Scalar, i.e. a Flag)
    %      A flag to indicate whether the current controller state is
    %      admissible, i.e. no error has occurred during computation of
    %      fresh input sequences.
    %
    % Literature: 
    %  	Florian Rosenthal, Markus Jung, Martina Zitterbart, and Uwe D. Hanebeck,
    %   CoCPN - Towards Flexible and Adaptive Cyber-Physical Systems Through Cooperation,
    %   Proceedings of the 2019 16th IEEE Annual Consumer Communications & Networking Conference,
    %   Las Vegas, Nevada, USA, January 2019.
    %      
    %   Markus Jung, Florian Rosenthal, and Martina Zitterbart,
    %   CoCPN-Sim: An Integrated Simulation Environment for Cyber-Physical Systems,
    %   Proceedings of the 2018 IEEE/ACM Third International Conference on Internet-of-Things Design and Implementation (IoTDI), 
    %   Orlando, FL, USA, April 2018.
    
    
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
    
    arguments
        handle
        timestamp(1,1) double {mustBePositive, mustBeInteger} % ensure that timestamp is internally treated as double
        paramStruct {validateParamStruct} = struct([]) % default, empty struct, validation function below
    end
    ncs = GetNcsByHandle(handle); % crashes if handle is invalid
      
    caPackets = [];
    scPackets = [];
    acPackets = [];
    %csPackets = [];
    packetBuffer = DataPacketBuffer.getInstance();
    % get the packets from the buffer and group by destination address
    % caPackets (actuator idx = 1)
    % controller idx = 2, sensor idx = 3
    incomingPackets = packetBuffer.getDataPackets(handle);
    if ~isempty(incomingPackets)
        for packet = incomingPackets
            switch packet.destinationAddress
                case 1
                    caPackets = [caPackets, packet]; %#ok
                case 2
                    switch packet.sourceAddress
                        case 1
                            % expected to be an ACK packet, so check
                            assert(packet.isAck, ...
                                'ncs_doLoopStep:InvalidACKPacket', ...
                                '** Packet from 1 (actuator) to 2 (controller) should be an ACK **');
                            acPackets = [acPackets, packet]; %#ok
                        case 3
                            scPackets = [scPackets, packet]; %#ok
                        otherwise
                            issueErrorInvalidDestinationAddress(packet.destinationAddress, packet.sourceAddress);
                    end
                case 3
                     % so far, do nothing
                     %csPackets = [csPackets, packet];
                otherwise
                   issueErrorInvalidDestinationAddress(packet.destinationAddress); 
            end
        end
        packetBuffer.clear(handle);
    end   
    
    if isfield(paramStruct, 'samplingInterval')
        ncs.changeControllerSamplingInterval(paramStruct.samplingInterval);
    end
    
%     if isfield(paramStruct, 'sensorMeasDelta') && ncs.sensor.isEventBased
%         ncs.sensor.measurementDelta = paramStruct.sensorMeasDelta;
%     end
    if isfield(paramStruct, 'caDelayProbs')
        ncs.changeControllerCaDelayProbs(paramStruct.caDelayProbs);
    end
    if isfield(paramStruct, 'scDelayProbs')
        ncs.changeControllerScDelayProbs(paramStruct.scDelayProbs);
    end
%     if isfield(paramStruct, 'controlSequenceLength') 
%         ncs.changeControllerSequenceLength(double(paramStruct.controlSequenceLength));
%     end
   
    [controllerActuatorPacket, sensorControllerPacket, controllerAcks] ...
        = ncs.step(double(timestamp), scPackets, caPackets, acPackets);

    pktsOut = constructPacketsToSend(controllerActuatorPacket, sensorControllerPacket, controllerAcks);

    % also, forward some gathered data
    [stats.actual_control_error, stats.estimated_control_error] = ncs.getCurrentControlError();
    stats.actual_stagecosts = ncs.getCurrentStageCosts();
    
    % determine if sensor and/or controller do not send this time
    % determine if actuator sent out an ACK
    stats.sc_sent = ~isempty(sensorControllerPacket);
    stats.ca_sent = ~isempty(controllerActuatorPacket);
    stats.ac_sent = ~isempty(controllerAcks);
    % record the delays of the processed packets: column vector or empty matrix
    stats.sc_delays = arrayfun(@(p) p.packetDelay, scPackets)';
    stats.ca_delays = arrayfun(@(p) p.packetDelay, caPackets)';
    stats.ac_delays = arrayfun(@(p) p.packetDelay, acPackets)';
        
    % get the QoC, as perceived by the controller, to be used by the
    % congestion control
    [~, qocOut] = ncs.getCurrentQualityOfControl();
    
    controllerStateAdmissible = ncs.isControllerStateAdmissible();
end

%% constructPacketsToSend
function packetsOut = constructPacketsToSend(controllerActuatorPacket, sensorControllerPacket, controllerAcks)
    packets = controllerAcks;
    if ~isempty(controllerActuatorPacket)
        packets = [packets; controllerActuatorPacket];
    end
    if ~isempty(sensorControllerPacket)
        packets = [packets; sensorControllerPacket];
    end
    packetsOut = arrayfun(@(p) p, packets, 'UniformOutput', false);
end

%% issueErrorInvalidDestinationAddress
function issueErrorInvalidDestinationAddress(destinationAddress, sourceAddress)
    if nargin == 1
        error('ncs_doLoopStep:InvalidDestinationAddress', ...
            '** Unsupported destination address encountered (%d) **', destinationAddress);
    end
    error('ncs_doLoopStep:InvalidDestinationAddress', ...
        '** Unsupported destination address encountered (%d) for source address (%d) **', ...
        destinationAddress, sourceAddress); 

end

%% validateParamStruct
function validateParamStruct(paramStruct)
    assert(isempty(paramStruct) || (isstruct(paramStruct) && isscalar(paramStruct)), ...
        'ncs_doLoopStep:InvalidParamStruct', ...
        '** If <paramStruct> is present, it must be a single struct (might be empty, though) **'); 
end
