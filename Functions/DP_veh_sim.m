function sim = DP_veh_sim(in, bounds)
%For all points in the speed space, the acceleartion options are determined
%and the resulting speed, duration and energy consumption are calculated

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