function [v2, dt, E, a] = DP_carmodel(v1)
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
% ------------
% Output:   - v2: new vehicle speeds as array
%           - dt: distance step duration as array
%           - E: energy consumption as array
%           - a: acceleration options as array
% ------------
global m fr A cw eta_tr eta_re amax g rho ds a_res rounding

Froll = m*g*fr;
cdrag = 0.5*rho*cw*A;

a_coast = -(Froll+cdrag*v1^2)/m;
a_stop = -v1^2/2/ds;
amin = max(a_stop,-amax);
if a_coast>amin
    a = sort([linspace(amin, amax, a_res-1), a_coast]);
else
    a = linspace(amin, amax, a_res);
end
a(find(a>=0,1)) = 0; %ensure that the algorithm has the option to go straight

v2_raw = sqrt(v1^2+2*a*ds);
v2 = fix(v2_raw*10^rounding)*10^-rounding;
dt = 2*ds./(v1+v2);

Ewheel  = 0.5*m*(v2_raw.^2-v1^2)+Froll*ds+cdrag*...
    (0.25*a.^3.*dt.^4 + v1*a.^2.*dt.^3 + 1.5*a.*v1^2.*dt.^2 + v1^3.*dt);
E = Ewheel.*((Ewheel>0)./eta_tr+(Ewheel<=0)*eta_re);

end