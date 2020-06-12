function sim = DP_veh_sim(bounds)
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
% Input:    - bounds: struct containing the route boundary conditions
% ------------
% Output:   - sim: struct containting the new speed, distance step
%           duration and energy consumption for all possible velocities and
%           accelerations
% ------------

global v_res a_res

%% Simulate vehicle for speeds in vspace
vset = linspace(0,max(bounds.v),v_res);

a = zeros(v_res-1,a_res);
v2 = zeros(v_res-1,a_res);
dt = zeros(v_res-1,a_res);
E = zeros(v_res-1,a_res);
for i = 1:length(vset)
    [v2(i,:), dt(i,:), E(i,:), a(i,:)] =  DP_carmodel(vset(i));
end

sim.vspace.v = vset;
sim.vspace.a = a;
sim.vspace.v2 = v2;
sim.vspace.dt = dt;
sim.vspace.E = E;

%% Simulate vehicle for speeds on the speed boundary
vset_bounds = unique(bounds.v);

a = zeros(length(vset_bounds),a_res);
v2 = zeros(length(vset_bounds),a_res);
dt = zeros(length(vset_bounds),a_res);
E = zeros(length(vset_bounds),a_res);
for i = 1:length(vset_bounds)
    [v2(i,:), dt(i,:), E(i,:), a(i,:)] =  DP_carmodel(vset_bounds(i));
end

sim.bounds.v = vset_bounds;
sim.bounds.a = a;
sim.bounds.v2 = v2;
sim.bounds.dt = dt;
sim.bounds.E = E;
end