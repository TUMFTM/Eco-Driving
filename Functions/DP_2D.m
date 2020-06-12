function Res = DP_2D(Route, bounds, sim)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: finds the optimal speed profile taking the time space into 
% account
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - bounds: struct containing the route boundary conditions
%           - sim: struct containting the new speed, distance step
%           duration and energy consumption for all possible velocities and
%           accelerations
% ------------
% Output:   - c_time: time penalty that results in the minimum difference
% between the target time and the trip duration
% ------------

global ds t_res a_res

%% Permute sim results

%Vspace
sim.vspace.v2 = permute(repmat(sim.vspace.v2,1,1,t_res),[3 1 2]);
sim.vspace.dt = permute(repmat(sim.vspace.dt,1,1,t_res),[3 1 2]);
sim.vspace.E  = permute(repmat(sim.vspace.E ,1,1,t_res),[3 1 2]);

%Vbounds
sim.bounds.v2 = permute(repmat(sim.bounds.v2,1,1,t_res),[3 1 2]);
sim.bounds.dt = permute(repmat(sim.bounds.dt,1,1,t_res),[3 1 2]);
sim.bounds.E  = permute(repmat(sim.bounds.E ,1,1,t_res),[3 1 2]);

%% DP backward
v_space{length(bounds.s)} = 0;
Cost{length(bounds.s)} = zeros(1,t_res);
H = waitbar(0,'Progress');
Iupdate = round(linspace(1,length(bounds.s),100));

for i = length(bounds.s)-1:-1:1
    if ismember(i,Iupdate)
        waitbar((length(bounds.s)-i)/(length(bounds.s)-1),H)
    end
    
    I = find(sim.vspace.v<bounds.v(i));
    Ibound = sim.bounds.v==bounds.v(i);
    
    v_space{i} = [sim.vspace.v(I) sim.bounds.v(Ibound)];
    v2 = [sim.vspace.v2(:,I,:) sim.bounds.v2(:,Ibound,:)];
    dt = [sim.vspace.dt(:,I,:) sim.bounds.dt(:,Ibound,:)];
    E = [sim.vspace.E(:,I,:) sim.bounds.E(:,Ibound,:)];
    
    t2 =  repelem(bounds.t_space(i,:)',1,length(v_space{i}),a_res) + dt;
    if unique(v_space{i+1}) == 0 %Special case if vehicle has to stop
        v_space{i} = v_space{i}(2:end);
        t2 = t2(:,2:end,:) + Route.t_stop(bounds.s_stop==bounds.s(i+1));
        v2 = v2(:,2:end,:);
        E = E(:,2:end,:);
        Cost_next = NaN*ones(size(t2));                
        Cost_next(v2==0) = interp1(bounds.t_space(i+1,:),Cost{i+1},t2(v2==0));
    elseif any(bounds.s_tl == bounds.s(i+1)) %Special case if the next step is a traffic light
        Cost_next = interp2(bounds.t_space(i+1,:),v_space{i+1},Cost{i+1}',t2,v2);
        I_tl = find(bounds.s_tl==bounds.s(i+1));
        tred = bounds.t_redswitches{I_tl};
        tgreen = bounds.t_greenswitches{I_tl};
        for j = 1:length(tred) %Set all options ending in the redlight zone to nan
            Cost_next(and(t2>tred(j),t2<tgreen(j))) = NaN;
        end
    else
        Cost_next = interp2(bounds.t_space(i+1,:),v_space{i+1},Cost{i+1}',t2,v2);
    end
    
    Costs = E + Cost_next;
    Cost{i} = min(Costs,[],3);
    Cost{i}(isnan(Cost{i})) = max(Cost{i},[],'all');
    
    %Include the benefit of stopping
    for j = 1:t_res
        Cost{i}(j,1) = min(Cost{i}(j:end,1));
    end    
end
close(H)
%% DP forward

%Find best starting point
I_tmin = find(Cost{i} == min(Cost{i}),1,'last');
if I_tmin>1
    s = [0 0];
    t = [0 bounds.t_space(1,I_tmin)];
    v = [0 0];
else
    s = 0;
    t = 0;
    v = 0;
end

%Find optimal speed profile
for i = 1:length(bounds.s)-1
    
    if unique(v_space{i+1})==0
        s_new = [s(end) s(end)] + ds;
        t_new = t(end) + 2/v(end) + [0 Route.t_stop(bounds.s_stop==bounds.s(i+1))];      
        v_new = [0 0];
        
        %Allow stopping longer
        I_t = find(bounds.t_space(i+1,:)<=t_new(end),1,'last');
        tmin = bounds.t_space(i+1,find(Cost{i+1} == min(Cost{i+1}(I_t:end)),1,'last'));       
        t_new(end) = max(tmin,t_new(end));
    else        
        [v2set, dtset, Eset] =  DP_carmodel(v(end));
        tset = t(end)+dtset;
        
        Cnext = interp2(bounds.t_space(i+1,:),v_space{i+1},Cost{i+1}',tset,v2set);
        Cset  = Cnext + Eset;
        [~, Imin] = min(Cset);
        
        if v2set(Imin)>0
            s_new = s(end)+ds;
            t_new = tset(Imin);
            v_new = v2set(Imin);
        else %Add extra point if the vehicle comes to a stop
            I_t = find(bounds.t_space(i+1,:)>(t(end)+dtset(Imin)),1);
            
            s_new = s(end)+[ds ds];
            t_new = [tset(Imin) bounds.t_space(i+1,find(Cost{i+1}(:,1) == min(Cost{i+1}(I_t:end,1)),1,'last'))];
            v_new = [0 0];
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

end