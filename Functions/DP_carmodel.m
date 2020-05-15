function [v2, dt, E, a] = DP_carmodel(v1, in, a_res)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: calculates the new vehicle speeds, distance step durations,
% energy consumption and acceleration options for a given speed and
% number of accelerations
% ------------
% Input:    - v1: initial vehicle velocity as double
%           - in: struct containing all global constants
%           - a_res: required resolution of acceleration vector as int
% ------------
% Output:   - v2: new vehicle speeds as array
%           - dt: distance step duration as array
%           - E: energy consumption as array
%           - a: acceleration options as array
% ------------

Froll = in.m*in.g*in.fr;
cdrag = 0.5*in.rho*in.cw*in.A;

a_coast = -(Froll+cdrag*v1^2)/in.m;
a_stop = -v1^2/2/in.ds;
amin = max(a_stop,-in.amax);
if a_coast>amin
    a = sort([linspace(amin, in.amax, a_res-1), a_coast]);
else
    a = linspace(amin, in.amax, a_res);
end
a(find(a>=0,1)) = 0; %ensure that the algorithm has the option to go straight

v2_raw = sqrt(v1^2+2*a*in.ds);
v2 = fix(v2_raw*10^in.round)*10^-in.round;
dt = 2*in.ds./(v1+v2);

Ewheel  = 0.5*in.m*(v2_raw.^2-v1^2)+Froll*in.ds+cdrag*...
    (0.25*a.^3.*dt.^4 + v1*a.^2.*dt.^3 + 1.5*a.*v1^2.*dt.^2 + v1^3.*dt);
E = Ewheel.*((Ewheel>0)./in.eta_tr+(Ewheel<=0)*in.eta_re);

end