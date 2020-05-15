function E_avg = Econs(t,v,in)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: calculates energy consumption based on speed over time and 
% vehicle parameters
% ------------
% Input:    - t: time as array
%           - v: speed as array
%           - in: struct containing all global constants
% ------------
% Output:   - E_avg: average energy consumption as double
% ------------



Froll = in.m*in.g*in.fr;
E = zeros(size(t));
for i = 1:length(t)-1
    dt = t(i+1)-t(i);
    a  = (v(i+1)-v(i))/dt;
    ds = (v(i+1)+v(i))/2*dt;
    
    cdrag = 0.5*in.rho*in.cw*in.A;
    Ewheel  = 0.5*in.m*(v(i+1)^2-v(i)^2)+Froll*ds+cdrag*...
        (0.25*a^3*dt^4 + v(i)*a^2*dt^3 + 1.5*a*v(i)^2*dt^2 + v(i)^3*dt);
    E(i) = Ewheel.*((Ewheel>0)./in.eta_tr+(Ewheel<=0)*in.eta_re);
end

E = cumsum(E);
s_tot = trapz(t,v);
E_avg = E(end)/3600/s_tot; %Consumption in kWh/km

end
