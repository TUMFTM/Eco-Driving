function [difference, Res] = DP_1D(Route, bounds, sim, c1, plottrue)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: finds the optimal speed profile for a given time penalty
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - bounds: struct containing the route boundary conditions
%           - sim: struct containting the new speed, distance step
%           duration and energy consumption for all possible velocities and
%           accelerations
%           - c1: time penalty as double
%           - plottrue: flag for plotting optimal speed profile as boolean
% ------------
% Output:   - difference: time difference with target time
%           - Res: struct containing the resulting speed profile
% ------------

global horizon ds v_res

%% DP backward
Cost{length(bounds.s)} = 0;
v_space{length(bounds.s)} = 0;
for i = length(bounds.s)-1:-1:1
    
    %find indices of speeds smaller than v_max
    I = find(sim.vspace.v<bounds.v(i)); %find initial speeds smaller than boundary speed
    Ibound = sim.bounds.v==bounds.v(i); %find boundary speed
    
    %Limit options to speeds <= boundary speed
    v_space{i} = [sim.vspace.v(I) sim.bounds.v(Ibound)];
    v2 = [sim.vspace.v2(I,:); sim.bounds.v2(Ibound,:)];
    dt = [sim.vspace.dt(I,:); sim.bounds.dt(Ibound,:)];
    E = [sim.vspace.E(I,:); sim.bounds.E(Ibound,:)];
    a = [sim.vspace.a(I,:); sim.bounds.a(Ibound,:)];
    
    %Calculate costs
    if unique(v_space{i+1}) == 0 %Special case if vehicle has to stop
        Cost{i}= E(v2==0)+c1*dt(v2==0); %Cost for options that result in stopping
        if plottrue
            a_opt{i} = a(v2==0);
        end
    else
        Cost_next = interp1(v_space{i+1},Cost{i+1},v2); %Interpolate costs for all new speed options
        Costs = E+Cost_next+c1*dt; %Calculate total cost
        [Cost{i}, I_opt] = min(Costs,[],2); %find minimum for each speed point
        if plottrue
            a_opt{i} = a(sub2ind(size(a),(1:length(v_space{i}))',I_opt));
        end
    end
end

%% DP forward

%Start conditions
t = 0;
v = 0;
s = 0;

%Find optimal speed profile
for i = 1:length(bounds.s)-1
    
    %find distance to and phase of closest traffic light
    [l_tl, isred, isyellow, t_green] = tl_scan(Route, bounds, s(end), t(end));
    l_stop = bounds.s_stop(find(bounds.s_stop>s(end),1))-s(end);
    
    %determine speed at next point
    if l_tl<=horizon && isred && l_tl<l_stop %traffic light in sight and is red and precedes the next stop
        [s_new, t_new, v_new] = tl_strategy(s(end), t(end), v(end), l_tl, isyellow, t_green);
    else %no traffic light in sight, or traffic light is green, or there is a stop before the traffic light
        [v2set, dtset, Eset] =  DP_carmodel(v(end));
                
        if unique(v_space{i+1})==0
            s_new = [s(end) s(end)] + ds;
            t_new = t(end) + 2*ds/v(end) + [0 Route.t_stop(bounds.s_stop==bounds.s(i+1))];      
            v_new = [0 0];      
        else
            Cnext = interp1(v_space{i+1},Cost{i+1},v2set);
            Cset  = Cnext + Eset + c1*dtset;
            [~, Imin] = min(Cset);
            
            s_new = s(end)+ds;
            t_new = t(end)+dtset(Imin);
            v_new = v2set(Imin);
        end
    end
    
    if not(isempty(Route.s_measured)) %check leading vehicle constraint
       [s_new, t_new, v_new] = ACC(s(end), t(end), v(end), s_new, t_new, v_new, Route);
    end
    
    s = [s s_new]; %#ok<AGROW>
    t = [t t_new]; %#ok<AGROW>
    v = [v v_new]; %#ok<AGROW>
end

Res.s = s(1:end-1);
Res.t = t(1:end-1);
Res.v = v(1:end-1);

difference = Res.t(end)-Route.tmax;

%% Plot
if plottrue
    figure
    hold on
    xlabel('Distance in meters')
    ylabel('Speed in km/h')
    h_lim = plot(bounds.s,bounds.v*3.6,':');
    s_step = 10;
    I_vmax = find(bounds.v==max(bounds.v),1); %Find index of distance step that has all speed options
    v_steps = 20;
    v_plot_tot = v_space{I_vmax}(ceil(linspace(1,v_res,v_steps)));
    for i = s_step:s_step:length(bounds.s)-s_step
        I_plot = (ismember(v_space{i},v_plot_tot));
        
        s_plot = (i-1)*ones(1,sum(I_plot));
        v_plot = v_space{i}(I_plot)*3.6;
        ds_plot = s_step/2*ds*ones(1,sum(I_plot));
        a_plot = a_opt{i}(I_plot)'; 
        
        h_acc = quiver(s_plot,v_plot,ds_plot,a_plot,0,'k');
    end
    h_opt = plot(Res.s,Res.v*3.6);
    if not(isempty(bounds.s_tl))
        h_tl = plot([bounds.s_tl; bounds.s_tl],[0 50],'r--');
        legend([h_lim h_acc h_opt h_tl(1)],'Maximum speed','Optimal-acceleration grid','Optimal-speed profile','Traffic light')
    else
        legend([h_lim h_acc h_opt],'Maximum speed','Optimal acceleration grid','Optimal speed profile')
    end
end

end