classdef (SharedTestFixtures={matlab.unittest.fixtures.PathFixture(...
            'libncs_matlab/matlab', 'IncludingSubfolders', true)}) ...
        ComponentMapTest < matlab.unittest.TestCase
    % Test cases for ComponentMap.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2021  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        component;
        componentName;
        component2;
        
        componentMapUnderTest;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(~)
            % required to destroy the singleton instance
            clear ComponentMap;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.componentName = 'TestComponent';
            this.component =  DummyNcsComponent(this.componentName);
            this.component2 = this;
            
            this.componentMapUnderTest = ComponentMap.getInstance();
            
            this.addTeardown(@() this.tearDown());
        end
        
        
    end
    
    methods (Test)        
        %% testAddComponentAlreadyPresent
        function testAddComponentAlreadyPresent(this)
            % add a component to the map, then try to add it again
            this.componentMapUnderTest.addComponent(this.component);
            
            expectedErrId = 'ComponentMap:ComponentAlreadyPresent';
            this.verifyError(@() this.componentMapUnderTest.addComponent(this.component), expectedErrId);
            
        end
        
        %% testAddComponent
        function testAddComponent(this)
            actualIndex = this.componentMapUnderTest.addComponent(this.component);
            expectedIndex = 1;
            
            this.verifyEqual(actualIndex, expectedIndex);
            this.verifySameHandle(this.componentMapUnderTest.getComponent(expectedIndex), this.component);
        end
        
        %% testRemoveComponentByIndexInvalidIndex
        function testRemoveComponentByIndexInvalidIndex(this)
            % first, add a component to the map (index is 1)
            index = this.componentMapUnderTest.addComponent(this.component);
            this.assertEqual(index, 1);
            
            % now attempt to call function with non-existing index
            expectedErrId = 'ComponentMap:InvalidIndex';
            this.verifyError(@() this.componentMapUnderTest.removeComponentByIndex(2), expectedErrId);
        end
        
        %% testRemoveComponentByIndex
        function testRemoveComponentByIndex(this)
            % first, add two components to the map (indices are 1, 2)
            index = this.componentMapUnderTest.addComponent(this.component);
            index2 = this.componentMapUnderTest.addComponent(this.component2);
            this.assertEqual(index, 1);
            this.assertEqual(index2, 2);
            
            % now attempt to remove the first element
            this.componentMapUnderTest.removeComponentByIndex(index);
            % the removed component should not be present anymore
            this.verifyFalse(this.componentMapUnderTest.containsComponent(this.component));
            this.verifyTrue(this.componentMapUnderTest.containsComponent(this.component2));
        end
        
        %% testRemoveComponentInvalidComponent
        function testRemoveComponentInvalidComponent(this)
           expectedErrId = 'ComponentMap:ComponentNotPresent';
           % add a component, and try to remove another one which is not
           % present in the map
           this.componentMapUnderTest.addComponent(this.component);
           invalidComponent = this.component2;
           this.verifyError(@() this.componentMapUnderTest.removeComponent(invalidComponent), expectedErrId);
        end
        
        %% testRemoveComponent
        function testRemoveComponent(this)
            % first, add two components to the map
            this.componentMapUnderTest.addComponent(this.component);
            this.componentMapUnderTest.addComponent(this.component2);
            
            % now attempt to remove the second element
            this.componentMapUnderTest.removeComponent(this.component2);
            % the removed component should not be present anymore
            this.verifyFalse(this.componentMapUnderTest.containsComponent(this.component2));
            this.verifyTrue(this.componentMapUnderTest.containsComponent(this.component));
        end
        
        %% testClear
        function testClear(this)
            % first, add two components to the map
            this.componentMapUnderTest.addComponent(this.component);
            this.componentMapUnderTest.addComponent(this.component2);
            
            this.componentMapUnderTest.clear();
            
            % neither element should be present anymore
            this.verifyFalse(this.componentMapUnderTest.containsComponent(this.component));
            this.verifyFalse(this.componentMapUnderTest.containsComponent(this.component2));
        end
        
        %% testGetComponentInvalidIndex
        function testGetComponentInvalidIndex(this)
            
            % now attempt to call function with non-existing index
            invalidIndex = 1;
            expectedErrId = 'ComponentMap:InvalidIndex';
            this.verifyError(@() this.componentMapUnderTest.getComponent(invalidIndex), expectedErrId);
        end
        
        %% testGetComponentInvalidType
        function testGetComponentInvalidType(this)
            index = this.componentMapUnderTest.addComponent(this.component2);
            % now attempt to call function incorrect expected type
            expectedErrId = 'ComponentMap:InvalidComponentType';
            invalidType = 'NetworkedControlSystem';
            this.verifyError(@() this.componentMapUnderTest.getComponent(index, invalidType), expectedErrId);
        end
        
        %% testGetComponent
        function testGetComponent(this)
            % add a component and try to obtain it
            expectedComponent = this.component;
            index = this.componentMapUnderTest.addComponent(expectedComponent); 
           
            actualComponent = this.componentMapUnderTest.getComponent(index);
            this.verifySameHandle(actualComponent, expectedComponent);
            
            % now try to obtain with type check
            expectedType = 'DummyNcsComponent';
            actualComponent = this.componentMapUnderTest.getComponent(index, expectedType);
            this.verifySameHandle(actualComponent, expectedComponent);
        end
        
        %% testGetComponentIndex
        function testGetComponentIndex(this)
            % add a component and try to obtain its index
            expectedComponent = this.component;

            expectedIndex = this.componentMapUnderTest.addComponent(expectedComponent); 
            
            actualIndex = this.componentMapUnderTest.getComponentIndex(expectedComponent);
            
            % compare indices as uint32
            this.verifyEqual(uint32(actualIndex), uint32(expectedIndex));
            % sanity check
            this.verifyTrue(this.componentMapUnderTest.containsComponent(expectedComponent));
            
            % no try to obtain the index of an element that is not buffered
            actualIndex = this.componentMapUnderTest.getComponentIndex(this.component2);
            
            this.verifyEqual(actualIndex, -1);
            % sanity check
            this.verifyFalse(this.componentMapUnderTest.containsComponent(this.component2));
        end
        
        %% testContainsComponent
        function testContainsComponent(this)
            % add a component and test whether it is contained
            expectedComponent = this.component;
            this.componentMapUnderTest.addComponent(expectedComponent);
            
            this.verifyTrue(this.componentMapUnderTest.containsComponent(expectedComponent));
            % sanity check
            this.verifyGreaterThan(this.componentMapUnderTest.getComponentIndex(expectedComponent), -1);
            
            % no try to test for a component that is not buffered
            newComponent = DummyNcsComponent(42);
            
            this.verifyFalse(this.componentMapUnderTest.containsComponent(newComponent));
            % sanity check
            this.verifyEqual(this.componentMapUnderTest.getComponentIndex(newComponent), -1);
        end
        
        %% testGetInstance
        function testGetInstance(this)
            newInstance = ComponentMap.getInstance();
            this.verifySameHandle(newInstance, this.componentMapUnderTest);
            
            clear ComponentMap;
            newInstance = ComponentMap.getInstance();
            this.verifyNotSameHandle(newInstance, this.componentMapUnderTest);
        end
    end
    
end

