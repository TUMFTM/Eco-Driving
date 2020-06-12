function c_time = DP_1D_timepenalty_E(Route, bounds, sim)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: finds the time penalty that minimizes energy consumption. 
% The strict eco-ACC can't minimize the difference to the target time, as 
% in the relaxed eco-ACC and dedicated lane use cases, since the maximum 
% inter-vehicle distance ensures that the vehicle always arrives close to 
% the target time. 
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - bounds: struct containing the route boundary conditions
%           - sim: struct containting the new speed, distance step
% ------------
% Output:   - c_time: time penalty that results in the minimum difference
% between the target time and the trip duration
% ------------

f = @(c_time) Emin(Route, bounds, sim, c_time); % Transform into a function where c_time is the only variable
options = optimset('Display','iter','TolX',10,'TolFun',1);
c_time = fminsearch(f,1e5,options);

end

function E = Emin(Route, bounds, sim, c_time)
[~, Res_mixed] = DP_1D(Route, bounds, sim, c_time, false);
E = Econs(Res_mixed.t,Res_mixed.v);

end