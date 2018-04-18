function [pktsOut, stats] = ncs_doLoopStep(handle, timestamp)
    % Performs a control loop (NCS) cycle in Matlab. 
    %
    % Parameters:
    %   >> handle (Key into ComponentMap, uint32)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem.
    %
    %   >> timestamp (Positive Integer)
    %      The current simulation time (in Omnet), in pico-seconds.   
    %
    % Returns:
    %   << pktsOut (Cell array, column-vector like, of DataPackets)
    %      The DataPackets resulting from the execution of the cycle to be
    %      sent (e.g., measurements, control sequence), row-wise arranged
    %      in a cell array.
    %
    %   << stats (Struct)
    %     Struct containing statistical data gathered during the execution
    %     of the control cycle. At least the following fields are present:
    %     -actual_qoc (Nonnegative integer), indicating the current Quality of Control
    %     -sc_delays (Column vector of nonnegative integers, might be empty), 
    %     describing the delays (in time steps) the processed DataPackets sent from the sensor experienced
    %     -ca_delays (Column vector of nonnegative integers, might be empty), 
    %     describing the delays (in time steps) the processed DataPackets sent from the controller experienced
    %     -ac_delays (Column vector of nonnegative integers, might be empty), 
    %     describing the delays (in time steps) the processed ACKs sent from the actuator experienced
    %     -sc_sent (Logical, i.e., a flag), indicating whether a
    %     packet (i.e., a measurement) was sent out by the sensor or not
    %     -ca_sent (Logical, i.e., a flag), indicating whether a
    %     packet (i.e., a control sequence) was sent out by the controller or not
    %     -ac_sent (Logical, i.e., a flag), indicating whether a
    %     packet (i.e., an ACK) was sent out by the actuator or not
    
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
    
    ncs = GetNcsByHandle(handle); % crashes if handle is invalid
    assert(Checks.isPosScalar(timestamp) && mod(timestamp, 1) == 0, ...
        'ncs_doLoopStep:InvalidTimestamp', ...
        '** <timestamp> expected to be positive integer **');
    
    caPackets = [];
    scPackets = [];
    acPackets = [];
    %csPackets = [];
    timestep = convertToTimeStep(ncs, timestamp);
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
            % store the delay the packet experienced (in time steps)
            packet.packetDelay = timestep - packet.timeStamp;
        end
        packetBuffer.clear(handle);
    end

    [controllerActuatorPacket, sensorControllerPacket, controllerAck] ...
        = ncs.step(timestep, scPackets, caPackets, acPackets);

    pktsOut = constructPacketsToSend(controllerActuatorPacket, sensorControllerPacket, controllerAck);
    
    stats.actual_qoc = ncs.getQualityOfControl(timestep);
    % determine if sensor and/or controller do not send this time
    % determine if actuator sent out an ACK
    stats.sc_sent = ~isempty(sensorControllerPacket);
    stats.ca_sent = ~isempty(controllerActuatorPacket);
    stats.ac_sent = ~isempty(controllerAck);
    % record the delays of the processed packets: column vector or empty matrix
    stats.sc_delays = arrayfun(@(p) p.packetDelay, scPackets)';
    stats.ca_delays = arrayfun(@(p) p.packetDelay, caPackets)';
    stats.ac_delays = arrayfun(@(p) p.packetDelay, acPackets)';
end

%% convertToTimeStep
function timestep = convertToTimeStep(ncs, timestamp)
    timestep = timestamp / ConvertToPicoseconds(ncs.samplingInterval);
end

%% constructPacketsToSend
function packetsOut = constructPacketsToSend(controllerActuatorPacket, sensorControllerPacket, controllerAck)
    packets = controllerAck;
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
            '** Unsupported destination address encountered (%d)  **', destinationAddress);
    end
    error('ncs_doLoopStep:InvalidDestinationAddress', ...
        '** Unsupported destination address encountered (%d)  for source address (%d) **', ...
        destinationAddress, sourceAddress); 

end
