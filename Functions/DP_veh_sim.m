function sim = DP_veh_sim(in, bounds)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: Determins the acceleration options for all points in the 
% speed space and calculates the resulting speed, duration and energy 
% consumption
% ------------
% Input:    - in: struct containing all global constants
%           - bounds: struct containing the route boundary conditions
% ------------
% Output:   - sim: struct containting the new speed, distance step
%           duration and energy consumption for all possible velocities and
%           accelerations
% ------------

%% Simulate vehicle for speeds in vspace
vset = linspace(0,max(bounds.v),in.v_res);

a = zeros(in.v_res-1,in.a_res);
v2 = zeros(in.v_res-1,in.a_res);
dt = zeros(in.v_res-1,in.a_res);
E = zeros(in.v_res-1,in.a_res);
for i = 1:length(vset)
    [v2(i,:), dt(i,:), E(i,:), a(i,:)] =  DP_carmodel(vset(i),in, in.a_res);
end

sim.vspace.v = vset;
sim.vspace.a = a;
sim.vspace.v2 = v2;
sim.vspace.dt = dt;
sim.vspace.E = E;

%% Simulate vehicle for speeds on the speed boundary
vset_bounds = unique(bounds.v);

a = zeros(length(vset_bounds),in.a_res);
v2 = zeros(length(vset_bounds),in.a_res);
dt = zeros(length(vset_bounds),in.a_res);
E = zeros(length(vset_bounds),in.a_res);
for i = 1:length(vset_bounds)
    [v2(i,:), dt(i,:), E(i,:), a(i,:)] =  DP_carmodel(vset_bounds(i),in, in.a_res);
end

sim.bounds.v = vset_bounds;
sim.bounds.a = a;
sim.bounds.v2 = v2;
sim.bounds.dt = dt;
sim.bounds.E = E;
end