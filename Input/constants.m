function input = constants(Powertrain)

    %Powertrain
    switch Powertrain
        case 'Electric'
            input.powertrain = 'Electric'; %Powertrain name
            input.eta_tr = 0.8; % Traction Efficiency [-]
            input.eta_re = 0.8; % Regeneration Efficiency [-]
        case 'Combustion'
            input.powertrain = 'Combustion'; %Powertrain name
            input.eta_tr = 0.4; % Traction Efficiency [-]
            input.eta_re = 0;   % Regeneration Efficiency [-]
        otherwise
            error('Selected powertrain not defined in constants')
    end

    %Vehicle
    input.m = 19000;    % Vehicle mass [kg]
    input.fr = 0.008;   % Rolling friction coefficient [-]
    input.A = 8;        % Front surface area [m^2]
    input.cw = 0.7;     % Drag coefficient [-]
    
    %Comfort
    input.amax = 1;     % Maximum comfortable acceleration [m/s/s]

    %Environment
    input.g = 9.81;     % gravitational acceleration [m/s/s]
    input.rho = 1.2;    % air density [kg/m/m/m]
    
    %Traffic light horizon
    input.horizon = 50; %Horizon [m]
    input.t_yellow = 5; %duration of yellow phase [s]
    
    %Intelligent driver model
    input.lzero = 2;    % static inter-vehicle distance [m]
    input.tr = 1;       % reaction time [s]
    input.lcar = 5;     % Car length [m]
    input.strict = false;% enables maximum vehicle following distance [boolean]
    
    %Optimization parameters
    input.ds = 1;       %distance step [m]
    input.t_res = 100;  %resolution of time space [-]
    input.v_res = 101;  %resolution of velocity space [-]
    input.a_res = 4;    %resolution of acceleration vector [-]
    input.a_res_FW =4; %resolution of acceleration vector in FW DP [-]
    input.round = 4;    %rounding to a set number of digits to prevent numeric errors [-]
    input.cmax = 1e6;   %maximal time penalty for 1D simulation
end