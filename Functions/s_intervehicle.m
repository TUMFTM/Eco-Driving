function [diff, ds_leader, smin, smax] = s_intervehicle(s1,t1,v1,v2,Route,in,minmax)
%Determine if the constraints are violated
s = s1+in.ds;
t = t1+2*in.ds/(v1+v2);
ds_leader = interp1(Route.t_measured,Route.s_measured,t)-s;
v_leader = interp1(Route.t_measured,Route.v_measured,t);
if isnan(ds_leader) %If the time is larger than the measured time
    ds_leader = Route.s_measured(end)-s; %Vehicle is at the end of the route
    v_leader = 0; %standing still
end

smin = v2*in.tr+v2.*(v2-v_leader)/2/in.amax;
smax = 10+v2+0.0825*v2^2;

switch minmax
    case 'min'
        diff = ds_leader-smin;
    case 'max'
        if in.strict
            diff = ds_leader-smax;
        else
            diff = -1;
        end
end

end
