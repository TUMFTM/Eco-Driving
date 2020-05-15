function c_time = DP_1D_timepenalty(Route, in, bounds, sim)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: finds the time penalty that minimizes the difference between
% the target time and the trip duration
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - bounds: struct containing the route boundary conditions
%           - sim: struct containting the new speed, distance step
% ------------
% Output:   - c_time: time penalty that results in the minimum difference
% between the target time and the trip duration
% ------------

%Find the time penalty that minimizes the difference between the target
%time and trip duration

%% Optimize time penalty in DP-1D
f = @(c_time) DP_1D(Route, in, bounds, sim, c_time, false); % Transform into a function where c_time is the only variable
options = optimset('Display','iter','TolX',1e-5); %optimization options
c_time = fzero(f,[0 in.cmax],options);

end