function [l_tl, isred, isyellow, tgreen] = tl_scan(Route, in, bounds, s, t)

I_tl = find(Route.s_tl>=s,1); %Index of closest traffic light
if isempty(I_tl) %If there are no more traffic lights ahead of the vehicle
    l_tl= inf;
    isred = false;
    isyellow = false;
    tgreen = inf;
else
    l_tl = Route.s_tl(I_tl)-s;%distance to nearest traffic light
    
    I_previousgreenswitch = find(bounds.t_greenswitches{I_tl}<=t,1,'last'); %Index of previous switch to green phase
    I_previousredswitch = find(bounds.t_redswitches{I_tl}<=t,1,'last'); %Index of previous switch to red phase
    t_previousgreenswitch = bounds.t_greenswitches{I_tl}(I_previousgreenswitch); %time of previous switch to green phase
    t_previousredswitch = bounds.t_redswitches{I_tl}(I_previousredswitch); %time of previous switch to red phase
    isred = t_previousredswitch>t_previousgreenswitch; %If the most recent switch was green to red, the tl is currently red
    
    isyellow = (t-t_previousredswitch)<in.t_yellow; %determine if the phase is amber

    tgreen =t_previousgreenswitch + Route.t_red(I_tl)+ Route.t_green(I_tl); %time at which the upcoming traffic light switches to green
end
    
end