classdef CacheTest < matlab.unittest.TestCase
    % Test cases for Cache.
    
    % >> This function/class is part of CoCPN-Sim
    %
    %    For more information, see https://github.com/spp1914-cocpn/cocpn-sim
    %
    %    Copyright (C) 2017-2018  Florian Rosenthal <florian.rosenthal@kit.edu>
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
        cacheLocation;
        componentToCache;
        filename;
    end
    
    methods (Access = private)
        %% tearDown
        function tearDown(~)
            Cache.clear();
            % required to destroy the singleton instance
            clear Cache;
        end
    end
    
    methods (TestMethodSetup)
        %% init
        function init(this)
            this.filename = 'test';
            this.componentToCache = DummyNcsComponent(42);
            
            import matlab.unittest.fixtures.WorkingFolderFixture;
            
            this.applyFixture(WorkingFolderFixture);
            this.cacheLocation = [pwd filesep 'cache'];

            %this.cacheUnderTest = Cache.getInstance();
            Cache.setLocation(this.cacheLocation);
            this.addTeardown(@() this.tearDown());
        end
    end
    
    methods (Test)
        %% testEnsureExists
        function testEnsureExists(this)
            % first, assert that the cache folder does not exist
            this.assertEqual(exist(this.cacheLocation, 'dir'), 0);
            
            Cache.ensureExists();
            % now the folder should be existent
            this.verifyEqual(exist(this.cacheLocation, 'dir'), 7);
            this.verifyTrue(isdir(this.cacheLocation));
        end
        
        %% testInsertInvalidComponent
        function testInsertInvalidComponent(this)
            expectedErrId = 'Cache:Insert:ComponentNotAHandle';
            
            invalidComponent = eye(4); % not a handle
            this.verifyError(@() Cache.insert(invalidComponent, this.filename), expectedErrId);
        end
        
        %% testInsert
        function testInsert(this)
            lookupInfo = Cache.insert(this.componentToCache, this.filename);
            
            % check if the result is a struct with the expected fields
            this.verifyTrue(isstruct(lookupInfo));
            this.verifySize(lookupInfo, [1 1]);
            
            this.verifyTrue(isfield(lookupInfo, 'name'));
            this.verifyTrue(isfield(lookupInfo, 'size'));
            this.verifyTrue(isfield(lookupInfo, 'date'));
            this.verifyEqual(numel(fieldnames(lookupInfo)), 3);
            % finally, check if the created cache file really exists
            this.verifyEqual(exist([this.cacheLocation filesep lookupInfo.name], 'file'), 2);
        end
        
        %% testLookupInvalidStruct
        function testLookupInvalidStruct(this)
            expectedErrId = 'Cache:Lookup:InvalidStruct';
            
            invalidStruct = nan; % not a struct
            this.verifyError(@() Cache.lookup(invalidStruct), expectedErrId);
        end
        
        %% testLookupInvalidField
        function testLookupInvalidField(this)
            expectedErrId = 'Cache:Lookup:InvalidField';
            
            invalidStruct.name = 'test';
            invalidStruct.size = 42;
            invalidStruct.d = 12;
            % date is missing in the struct
            this.verifyError(@() Cache.lookup(invalidStruct), expectedErrId);
        end
        
        %% testLookup
        function testLookup(this)
            % first, insert a component
            lookupInfo = Cache.insert(this.componentToCache, this.filename);
            % now attempt to load it from the cache
            actualComponent = Cache.lookup(lookupInfo);
            
            this.verifyEqual(actualComponent, this.componentToCache);
            this.verifyEqual(actualComponent.componentId, this.componentToCache.componentId);
        end
        
        %% testLookupNotFound
        function testLookupNotFound(this)
            % first, insert a component
            trueLookupInfo = Cache.insert(this.componentToCache, this.filename);
            
            % manipulate the file name
            wrongLookupInfo.name = 'test2.mat';
            wrongLookupInfo.size = trueLookupInfo.size;
            wrongLookupInfo.date = trueLookupInfo.date;
            
            % now attempt to load it from the cache
            actualComponent = Cache.lookup(wrongLookupInfo);
            
            this.verifyEmpty(actualComponent);
            
            % manipulate the size
            wrongLookupInfo.name = trueLookupInfo.name;
            wrongLookupInfo.size = trueLookupInfo.size + 1;
            wrongLookupInfo.date = trueLookupInfo.date;
            
            % now attempt to load it from the cache
            actualComponent = Cache.lookup(wrongLookupInfo);
            
            this.verifyEmpty(actualComponent);
            
            % manipulate the date
            wrongLookupInfo.name = trueLookupInfo.name;
            wrongLookupInfo.size = trueLookupInfo.size;
            wrongLookupInfo.date = trueLookupInfo.date + 1;
            
            % now attempt to load it from the cache
            actualComponent = Cache.lookup(wrongLookupInfo);
            
            this.verifyEmpty(actualComponent);
        end
        
        %% testClear
        function testClear(this)
             % first, assert that the cache folder does not exist
            this.assertEqual(exist(this.cacheLocation, 'dir'), 0);
            
            % now insert a component, and clear afterwards
            Cache.insert(this.componentToCache, this.filename);
            this.assertEqual(exist(this.cacheLocation, 'dir'), 7);
            this.assertTrue(isdir(this.cacheLocation));
            
            Cache.clear();
            
            this.verifyEqual(exist(this.cacheLocation, 'dir'), 0);
            this.verifyFalse(isdir(this.cacheLocation));
            
        end
        
        %% testSetLocationInvalidChar
        function testSetLocationInvalidChar(this)
            expectedErrId = 'Cache:SetLocation:InvalidChar';
            
            invalidLocation = this; % not a char
            this.verifyError(@() Cache.setLocation(invalidLocation), expectedErrId);
            
            invalidLocation = transpose('cache'); % not a row vector
            this.verifyError(@() Cache.setLocation(invalidLocation), expectedErrId);
        end
    end
    
end

