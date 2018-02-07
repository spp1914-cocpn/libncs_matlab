classdef NcsController < handle
    % Wrapper class for controllers in an NCS to provide a consistent
    % interface for controllers that require an external filter or state
    % estimate, and those which don't.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (SetAccess=immutable, GetAccess = protected)
        controller@SequenceBasedController;
    end
    
     properties (Dependent, GetAccess = public)
        controlSequenceLength;
     end
    
    methods
        function sequenceLength = get.controlSequenceLength(this)
            sequenceLength = this.controller.sequenceLength;
        end
    end
   
    methods (Access = public)
        %% NcsController
        function this = NcsController(controller)
            % Class constructor.
            %
            % Parameters:
            %   >> controller (SequenceBasedController instance)
            %      The controller to be utilized within the corresponding
            %      NCS.
            %
            % Returns:
            %   << this (NcsController)
            %      A new NcsController instance.
            this.controller = controller;
        end
        
        function [inputSequence, numUsedMeas, numDiscardedMeas] ...
                = step(this, timestep, scPackets, acPackets, plantMode)
            % Compute a control sequence as part of a control cycle in an
            % NCS.
            %
            % Parameters:
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %     
            %   >> scPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing measurements taken and transmitted from the sensor.
            %
            %   >> acPackets (Array of DataPackets, might be empty)
            %      An array of DataPackets containing ACKs returned from the actuator.
            %   
            %   >> plantMode (Nonnegative integer, might be empty)
            %      The previous true plant mode (theta_{k-1}), or the empty
            %      matrix, if not directly known to the controller.
            %
            % Returns:
            %   << inputSequence (Matrix of size dimInput x controlSequenceLength, might be empty)
            %      The new control sequence computed by the controller, with the individual inputs column-wise arranged.
            %      Empty matrix is returned in case none is to be transmitted (e.g., when the controller is event-based).
            %
            %   << numUsedMeas (Nonnegative integer)
            %      The number of measurements actually used for the
            %      computation of the control sequence.
            %
            %   << numDiscardedMeas (Nonnegative integer)
            %      The number of measurements which were discarded due to
            %      a too large delay.
            %
            [measurements, measDelays] = NcsController.processScPackets(scPackets);
            inputSequence = ...
                this.reshapeInputSequence(this.controller.computeControlSequence(measurements, measDelays));
            [numUsedMeas, numDiscardedMeas] = this.controller.getLastComputationMeasurementData();
        end
        
        %% computeCosts
        function controlCosts = computeCosts(this, plantStates, appliedInputs)
            controlCosts = this.controller.computeCosts(plantStates, appliedInputs);
        end
        
        %% getCurrentQualityOfControl
        function qoc = getCurrentQualityOfControl(this, plantState, timestep)
            % Get the current quality of control (QoC) for the given plant
            % true state.
            %
            % Parameters:
            %   >> trueState (Vector)
            %      The plant true state at the given time step.
            %
            %   >> timestep (Positive integer)
            %      The current time step, i.e., the integer yielding the
            %      current simulation time (in s) when multiplied by the
            %      loop's sampling interval.
            %
            % Returns:
            %   << currentQoC (Nonnegative scalar)
            %      The current QoC, which is simply the norm of the current
            %      plant true state, or, in a tracking task, the norm of
            %      the deviation from the current reference output.
            
            if Checks.isClass(this.controller, 'SequenceBasedTrackingController')
                % we track a reference
                qoc = norm(this.controller.getDeviationFromRefForState(plantState, timestep)); 
            else
                % we track the origin
                qoc = norm(plantState);
            end
        end
    end
  
    methods (Access = protected)
        %% reshapeInputSequence
        function sequence = reshapeInputSequence(this, inputSequence)
            % reshape the resulting stacked vector (arrange inputs column-wise)
            if ~isempty(inputSequence)
                sequence = reshape(inputSequence, [], this.controlSequenceLength);
            else
                sequence = [];
            end
        end
    end
    
    methods (Static, Access = protected)
        function [measurements, measDelays] = processScPackets(scPackets)
            measurements = [];
            measDelays = [];
            if numel(scPackets) ~= 0
                [delays{1:numel(scPackets)}] = scPackets(:).packetDelay;
                [meas{1:numel(scPackets)}] = scPackets(:).payload;
                measurements = cell2mat(meas);
                measDelays = cell2mat(delays);
            end
        end
    end
end

