function picoseconds = ConvertToPicoseconds(seconds)
    % Convenience function to convert a given time value in seconds into
    % pico-seconds.
    %
    % Parameters:
    %   >> seconds (Scalar)
    %      A scalar denoting a time value in seconds. 
    %
    % Returns:
    %   >> picoseconds (Scalar)
    %      The corresponding value in pico-seconds.

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
    
    picoseconds = seconds * 1e12;
end

