classdef Cache < handle
    % Singleton implementation of a cache for NCS components which can be
    % reused by multiple simulation runs, e.g., during parameter studies.
    % 
    % By default, this implementation assumes that the working folder
    % during simulation runs is ncs-testbench/simulations where omnetpp.ini
    % is located.
    
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
    
    properties (Access = private)
        cacheFolder;
    end
       
    methods (Access = private)
        function this = Cache()
            % assume that pwd is ncs-testbench/simulations
            this.setLocationInternal([pwd '/../../libncs_matlab/out/tmp']);
        end
        
        function setLocationInternal(this, newLocation)
            this.cacheFolder = newLocation;
        end
        
        function ensureExistsInternal(this)
           if ~isdir(this.cacheFolder)
                [status, message, ~] = mkdir(this.cacheFolder);
                if ~status
                    error('Cache:EnsureExists:Mkdir', message);
                end
            end 
        end
        
        function clearInternal(this)
            if isdir(this.cacheFolder)
                [status, message, ~] = rmdir(this.cacheFolder, 's');
                if ~status
                    error('Cache:Clear:Rmdir', message);
                end
            end 
        end
        
        function lookupInfo = insertInternal(this, ncsComponent, filename) %#ok
            this.ensureExistsInternal();
            filename = [this.cacheFolder '/' filename];
            
            save(filename, 'ncsComponent');
            attributes = dir(filename);
            lookupInfo.name = attributes.name;
            lookupInfo.date = attributes.datenum;
            lookupInfo.size = attributes.bytes;
        end
        
        function ncsComponent = lookupInternal(this, name, date, size)
            filename = [this.cacheFolder '/' name];
            ncsComponent = [];
            if exist(filename, 'file') == 2
                attributes = dir(filename);

                if attributes.datenum == date && size == attributes.bytes
                    data = load(filename);
                    ncsComponent = data.ncsComponent;
                end
            end
        end
    end
    
    methods (Static, Access = public)
        %% ensureExists
        function ensureExists()
            Cache.getInstance().ensureExistsInternal();
        end
        
        %% insert
        function lookup = insert(ncsComponent, name)
            if ~isa(ncsComponent, 'handle')
                error('Cache:Insert:ComponentNotAHandle', ...
                    '** Cannot insert component for it is not a handle **');
            end
            lookup = Cache.getInstance().insertInternal(ncsComponent, [name '.mat']);
        end
        
        %% lookup
        function ncsComponent = lookup(lookupInfo)
            if ~isstruct(lookupInfo)
                error('Cache:Lookup:InvalidStruct', ...
                     '** <lookupInfo> must be a structure array. **');
            end
            expectedFields = {'name', 'date', 'size'};
            found = isfield(lookupInfo, expectedFields);
            notFoundIdx = find(~found);
            if numel(notFoundIdx) > 0
                error('Cache:Lookup:InvalidField', ...
                    '** The following %d variables must be present in the lookup structure: %s **', ...
                    numel(notFoundIdx), strjoin(expectedFields(notFoundIdx), ','));
            end
            ncsComponent = Cache.getInstance().lookupInternal(lookupInfo.name, lookupInfo.date, lookupInfo.size);
        end
        
        %% clear
        function clear()
            Cache.getInstance().clearInternal();
        end
        
        %% setLocation
        function setLocation(cacheLocation)
            if ~isrow(cacheLocation) || ~ischar(cacheLocation)
                error('Cache:SetLocation:InvalidChar', ...
                    '** <cacheLocation> must be a char array (row vector-like). **');
            end
            Cache.getInstance().setLocationInternal(cacheLocation);
        end
    end
    
    methods (Static, Access = private)
        %% getInstance
        function instance = getInstance()
            persistent soleInstance
            if isempty(soleInstance) || ~isvalid(soleInstance)
                soleInstance = Cache();
            end
            instance = soleInstance;
        end
    end
    
end

