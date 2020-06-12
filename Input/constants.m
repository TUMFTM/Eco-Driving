global m fr A cw eta_tr eta_re amax g rho horizon t_yellow lzero tr ...
    lcar ds t_res v_res a_res rounding cmax

%Vehicle
m = 19000;    % Vehicle mass [kg]
fr = 0.008;   % Rolling friction coefficient [-]
A = 8;        % Front surface area [m^2]
cw = 0.7;     % Drag coefficient [-]

%Powertrain
eta_tr = 0.4; % Traction Efficiency [-]
eta_re = 0;   % Regeneration Efficiency [-]

%Comfort
amax = 1;     % Maximum comfortable acceleration [m/s/s]

%Environment
g = 9.81;     % gravitational acceleration [m/s/s]
rho = 1.2;    % air density [kg/m/m/m]

%Traffic light
horizon = 50; %Horizon [m]
t_yellow = 5; %duration of yellow phase [s]

%Intelligent driver model
lzero = 2;    % static inter-vehicle distance [m]
tr = 1;       % reaction time [s]
lcar = 5;     % Car length [m]

%Optimization parameters
ds = 1;       %distance step [m]
t_res = 100;  %resolution of time space [-]
v_res = 101;  %resolution of velocity space [-]
a_res = 4;    %resolution of acceleration vector [-]
rounding = 4;    %rounding to a set number of digits to prevent numeric errors [-]
cmax = 1e6;   %maximal time penalty for 1D simulation