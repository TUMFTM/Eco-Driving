function bounds = DP_1D_bounds(Route, in)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: returns the speed bound over time based on the location of
% stops, traffic lights and acceleration limits. Additionally, the phase
% changes of all traffic lights during the trip duration are calculated
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
% ------------
% Output:   - bounds: struct containing the route boundary conditions
% ------------

%% Generate distance vector and adjust location of stops and traffic lights
bounds.s = 0:in.ds:Route.s_vlim(end);
bounds.s_tl = [];
bounds.s_stop = [];
for i = length(Route.s_stop):-1:1
    bounds.s_stop(i) = bounds.s(find(bounds.s<=Route.s_stop(i),1,'last')); %adjust for distance step resolution
end
for i = length(Route.s_tl):-1:1
    bounds.s_tl(i) = bounds.s(find(bounds.s<=Route.s_tl(i),1,'last')); %adjust for distance step resolution
end

%% generate speed bound profile without acceleration and deceleration phases
vlim = interp1(Route.s_vlim,Route.v_vlim,bounds.s,'previous');
s_corner = round(Route.s_corner/in.ds)*in.ds; %adjust for distance step resolution
vlim(ismember(bounds.s,bounds.s_stop)) = 0; %set speed limit at stops
vlim(ismember(bounds.s,s_corner)) = Route.v_corner; %set speed limit at corners

%% generate distance vector for acceleration and deceleration phases
vmax = max(Route.v_vlim);
smax = vmax^2/2/in.amax;
s = 0:in.ds:smax;

%% adjust speed bound for acceleration and deceleration phases
Iacc = find(diff(vlim)>0); %Find distance where acceleration starts
Idec = find(diff(vlim)<0)+1; %Find distance where deceleration ends
v_matrix = NaN*ones(length(Iacc)+length(Idec)+2,length(bounds.s)); %Create matrix for curves
v_matrix(end,:) = vlim; %speed limit curve

%acceleration phases
for i = 1:length(Iacc) 
    v_matrix(i,Iacc(i):Iacc(i)+length(s)-1) = sqrt(vlim(Iacc(i))^2+2*in.amax*s);
end

%deceleration phases
for i = 1:length(Idec)
    v_matrix(i+length(Iacc)+1,Idec(i)-length(s)+1:Idec(i))= sqrt(vlim(Idec(i))^2+2*in.amax*(s(end)-s));
end

v_matrix = v_matrix(:,1:length(bounds.s)); %Cut off length added by too long acc or dec profiles
bounds.v = fix(min(v_matrix)*10^in.round)*10^-in.round; %Derive speed bound

%% Determine traffic light switching times 
bounds.t_greenswitches = cell(1,length(Route.s_tl));
bounds.t_redswitches = cell(1,length(Route.s_tl));
for i = 1:length(Route.s_tl)
    cycle_duration = Route.t_green(i)+Route.t_red(i);
    bounds.t_greenswitches{i} = Route.t_firstgreen(i)+(-1:ceil(Route.tmax/cycle_duration))*cycle_duration;
    bounds.t_redswitches{i} = bounds.t_greenswitches{i}(1:end)-Route.t_red(i);
end

end