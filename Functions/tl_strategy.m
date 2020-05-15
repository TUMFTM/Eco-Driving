function [s_new, t_new, v_new] = tl_strategy(s1, t1, v1, l_tl, in, isyellow, t_green)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: Determines traffic light approach strategy
% ------------
% Input:    - s1: initial distance as double
%           - t1: initial time as double
%           - v1: initial speed as double
%           - l_tl: distance to upcoming traffic light as double
%           - in: struct containing all global constants
%           - isyellow: flag if traffic light is yellow as boolean
%           - tgreen: time at which the upcoming traffic light switches to 
% ------------
% Output:   - s_new: new distance as double
%           - t_new: new time as double
%           - v_new: new speed as double
% ------------

Froll = in.m*in.g*in.fr; %Rolling resistance [N]
Fdrag = 0.5*in.rho*in.cw*in.A*v1^2; %Drag resistance [N]

if l_tl > in.ds
    vdec = sqrt(2*in.amax*l_tl); %max. deceleration speed
    vcoast = sqrt(2*l_tl*Froll./(in.m-l_tl*in.rho*in.cw*in.A)); %Coasting speed
    if v1<vcoast %Zone I: If the vehicle is too slow to coast to a halt
        vmax = sqrt(v1^2+2*in.amax*in.ds); 
        v_new = min(vmax,vcoast); %accelerate with a_max until coasting speed
    elseif v1>=vcoast && v1<=vdec % Zone II
        a = -(Froll+Fdrag)/in.m; %coasting deceleartion
        vnew_coast = sqrt(v1^2+2*a*in.ds); % new coasting speed
        vnew_dec = sqrt(2*in.amax*(l_tl-in.ds)); % max. deceleration speed
        v_new = min(vnew_coast,vnew_dec);
    else %Zone III
        if isyellow
            v_new = v1; %maintain speed
        else
            a_req = -v1^2/2/l_tl; %required deceleration 
            v_new = real(sqrt(v1^2+2*a_req*in.ds)); %brake
        end
    end
    t_new = t1+2/(v1+v_new);
    s_new = s1+in.ds;
else %If the next step is the traffic light
    if isyellow
        v_new = v1;
        s_new = s1+in.ds;
        t_new = t1+2/(v1+v_new);
    else
        s_new = [s1 s1] + in.ds;
        t_new = [t1+2/v1 t_green];
        v_new = [0 0];        
    end
end

end