function lastNumCompThreads = ncs_setSingleThreaded()
    % Set the number of computational threads used by Matlab to 1, thus
    % demanding a single threaded computation.
    %
    % Parameters:
    %   >> Returns (Positive integer)
    %      The previous maximum number of computational threads.
    %       By default, the maximum number of computational threads is
    %       equal to the number of physical cores.
    
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
    
    lastNumCompThreads = maxNumCompThreads(1); % simulation shall be run single threaded
end

