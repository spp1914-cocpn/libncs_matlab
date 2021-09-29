function plantInterval = ncs_getPlantTickerInterval(handle)
    % Gets the ticker interval (sampling interval) (in pico-seconds) of the plant a networked control
    % system, i.e., the period between two invocations of the plant
    % dynamics.
    %
    % Parameters:
    %   >> handle (Key into ComponentMap)
    %      A handle (key into ComponentMap) which uniquely identifies a NetworkedControlSystem instance.
    %
    % Returns:
    %   << plantInterval (Nonnegative integer)
    %      The ticker interval, expressed in pico-seconds. 
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
    
    ncs = GetNcsByHandle(handle);

    % translate sampling interval (given in seconds) into pico-seconds
    plantInterval = ConvertToPicoseconds(ncs.plantSamplingInterval);
end

