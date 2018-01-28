function ncs_seedRng(seed)
    % Control generation of random numbers in Matlab to produce a
    % predictable sequence of numbers.
    %
    % Parameters:
    %   >> seed (Nonnegative integer)
    %      The seed to use. Matlab by default uses seed 0.
    %
    %
    % Returns:
    %   << packet (DataPacket)
    %      The created DataPacket instance.  
    
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
    
    if ~Checks.isNonNegativeScalar(seed) || mod(seed, 1) ~= 0
        error('ncs_seedRng:InvalidSeed', ...
          '** <seed> expected to be a nonnegative integer **'); 
    end
    rng(seed);
end

