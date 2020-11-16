classdef (Sealed) ComponentMap < handle
    % Singleton implementation of a map for NCS components which allows for
    % associating and components with nonnegative indices/ids.
    %
    % Main purpose of this class is to provide a means to identify NCS in
    % Omnet. 
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2020  Florian Rosenthal <florian.rosenthal@kit.edu>
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
    
    properties (Access = private)
        % maps indices (considered as unique addresses) to components
        componentMap;
    end
    
    properties (Constant)
        maxIndex = intmax('uint32');
    end
    
    methods (Access = private)
        %% ComponentMap
        function this = ComponentMap()
            this.componentMap = containers.Map('KeyType', 'uint32', 'ValueType', 'any');
        end
        
        %% checkIndex
        function checkIndex(this, index)
            assert(this.componentMap.isKey(index), ...
                'ComponentMap:InvalidIndex', ...
                '** Given index %d is invalid **', index);
        end
    end
    
    methods (Access = public)
        %% addComponent
        function index = addComponent(this, component)
            % Add an NCS component to the map.
            %
            % Parameters:
            %   >> component (Handle)
            %      A component of an NCS, or a complete NCS, i.e., a
            %      NetworkedControlSystem instance.
            %
            % Returns:
            %   << index (Nonnegative integer)
            %      The index which can be used to retrieve the component
            %      from the map.
            arguments
                this
                component(1,1) handle
            end
      
            assert(~this.containsComponent(component), ...
                'ComponentMap:ComponentAlreadyPresent', ...
                '** Cannot add component for it is already contained in the map **');
            
            index = ComponentMap.getNewIndex();
            this.componentMap(index) = component;
        end
        
        %% removeComponentByIndex
        function removeComponentByIndex(this, index)
            % Remove a component from the map.
            %
            % Parameters:
            %   >> index (Nonnegative integer)
            %      The index of the component to remove from the map.

            this.checkIndex(index);
            
            this.componentMap.remove(index);
        end
        
        %% removeComponent
        function removeComponent(this, component)
            % Remove a component from the map.
            %
            % Parameters:
            %   >> component (Handle)
            %      The component to remove from the map.
            
            index = this.getComponentIndex(component);
            assert(index ~= -1, ...
                'ComponentMap:ComponentNotPresent', ...
                '** Cannot remove component: It is not present in the map **');
            
            this.componentMap.remove(index);
        end
        
        %% clear
        function clear(this)
            % Clears the map, i.e., removes all stored components.
            %
            this.componentMap.remove(this.componentMap.keys());
        end
        
        %% getComponent
        function component = getComponent(this, index, expectedTypeName)
            % Retrieve a component from the map by index.
            %
            % Parameters:
            %   >> index (Nonnegative integer)
            %      The index of the component to retrieve.
            %
            %   >> expectedTypeName (Char array, optional)
            %      If a type name is passed here, the type of the component
            %      is checked and must match that specified by the given
            %      name.
            %
            % Returns:
            %   << component (Handle)
            %      A component of an NCS, or a complete NCS, i.e., a
            %      NetworkedControlSystem instance.
            
            this.checkIndex(index);
            
            component = this.componentMap(index);
            if nargin == 3 && ~Checks.isClass(component, expectedTypeName)
                error('ComponentMap:InvalidComponentType',  ...
                    '** Expected type of component is %s, but true type is %s **', ...
                    expectedTypeName, class(component));
            end
        end
        
        %% getComponentIndex
        function index = getComponentIndex(this, component)
            % Retrieve the index of a component stored in the map.
            %
            % Parameters:
            %   >> component (Handle)
            %      A component of an NCS, or a complete NCS, i.e., a
            %      NetworkedControlSystem instance.
            %
            % Returns:
            %   >> index (Integer)
            %      The index of the component, if present in the map, -1
            %      otherwise.
            
            % use == to check for identity of handles
            index = -1;
            for key = this.componentMap.keys()
                if this.componentMap(key{:}) == component
                    index = key{:};
                    break;
                end
            end
        end
        
        %% containsComponent
        function result = containsComponent(this, component)
            % Check whether the given component is contained in the map.
            %
            % Parameters:
            %   >> component (Handle)
            %      A component of an NCS, or a complete NCS, i.e., a
            %      NetworkedControlSystem instance.
            %
            % Returns:
            %   >> result (Logical)
            %      A logical indicating whether the given component is
            %      present in the map or not.
            
            % use == to check for identity of handles
            result = false;
            for c = this.componentMap.values()
                if c{:} == component
                    result = true;
                    break;
                end
            end
        end
    end
    
    methods (Static, Access = private)
        function newIndex = getNewIndex()
            persistent componentCounter
            if isempty(componentCounter)
                componentCounter = 1;
            elseif componentCounter == ComponentMap.maxIndex
                 error('ComponentMap:MaxCapacity', ...
                    '** Maximum capacity (%d) has been reached, cannot compute new index **', ...
                    ComponentMap.maxIndex);
            else
                componentCounter = componentCounter + 1;
            end
            newIndex = componentCounter;
        end
    end
    
    methods (Static, Access = public)
        function instance = getInstance()
            % Access the singleton instance of this class.
            %
            %
            % Returns:
            %   << instance (ComponentMap)
            %      The sole instance of this class.
            
            persistent soleInstance
            if isempty(soleInstance) || ~isvalid(soleInstance)
                soleInstance = ComponentMap();
            end
            instance = soleInstance;
        end
    end
    
end

