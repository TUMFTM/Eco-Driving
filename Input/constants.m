function Input = constants(Powertrain)

    %Powertrain
    switch Powertrain
        case 'Electric'
            Input.powertrain = 'Electric'; %Powertrain name
            Input.eta_tr = 0.8; % Traction Efficiency [-]
            Input.eta_re = 0.8; % Regeneration Efficiency [-]
        case 'Combustion'
            Input.powertrain = 'Combustion'; %Powertrain name
            Input.eta_tr = 0.4; % Traction Efficiency [-]
            Input.eta_re = 0;   % Regeneration Efficiency [-]
        otherwise
            error('Selected powertrain not defined in constants')
    end

    %Vehicle
    Input.m = 19000;    % Vehicle mass [kg]
    Input.fr = 0.008;   % Rolling friction coefficient [-]
    Input.A = 8;        % Front surface area [m^2]
    Input.cw = 0.7;     % Drag coefficient [-]
    
    %Comfort
    Input.amax = 1;     % Maximum comfortable acceleration [m/s/s]

    %Environment
    Input.g = 9.81;     % gravitational acceleration [m/s/s]
    Input.rho = 1.2;    % air density [kg/m/m/m]
    
    %Traffic light horizon
    Input.horizon = 50; %Horizon [m]
    Input.t_yellow = 5; %duration of yellow phase [s]
    
    %Intelligent driver model
    Input.lzero = 2;    % static inter-vehicle distance [m]
    Input.tr = 1;       % reaction time [s]
    Input.lcar = 5;     % Car length [m]
    Input.strict = false;% enables maximum vehicle following distance [boolean]
    
    %Optimization parameters
    Input.ds = 1;       %distance step [m]
    Input.t_res = 100;  %resolution of time space [-]
    Input.v_res = 101;  %resolution of velocity space [-]
    Input.a_res = 4;    %resolution of acceleration vector [-]
    Input.a_res_FW =4; %resolution of acceleration vector in FW DP [-]
    Input.round = 4;    %rounding to a set number of digits to prevent numeric errors [-]
    Input.cmax = 1e6;   %maximal time penalty for 1D simulation
end