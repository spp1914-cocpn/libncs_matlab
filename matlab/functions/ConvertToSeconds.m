function seconds = ConvertToSeconds(picoseconds)
    % Convenience function to convert a given time value in pico-seconds into
    % seconds.
    %
    % Parameters:
    %   >> picoseconds (Scalar)
    %      A scalar denoting a time value in pico-seconds. 
    %
    % Returns:
    %   >> seconds (Scalar)
    %      The corresponding value in seconds.

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

    seconds = picoseconds / 1e12;
end

