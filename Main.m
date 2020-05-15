close all
clearvars
clc
addpath(genpath(pwd))

% Author: Olaf Teichert
% Date: 2020-02-24

%% Input
Route = loadroute('Route91_first3stops'); % load boundary conditions
in = constants('Combustion'); % load remaining constants

%Dedicated lane
Route_dl = Route; %Create special case for dedicated lane
Route_dl.s_measured = []; %by removing the leading vehicle

%Tight vehicle following
in_strict = in; %Create special case for tight vehicle following
in_strict.strict = true; %by enabling tight vehicle following

%% Preprocessing
bounds_1D = DP_1D_bounds(Route, in); %bounds and traffic lights for 1D state space
bounds_2D = DP_2D_bounds(Route, in, bounds_1D, false); %bounds and traffic lights for 2D state space
sim = DP_veh_sim(in, bounds_1D); %vehicle simulation

%% Mixed Traffic relaxed eco-ACC
c_relaxed = DP_1D_timepenalty(Route,in, bounds_1D, sim); %find time penalty
[~, Res_relaxed] = DP_1D(Route, in, bounds_1D, sim, c_relaxed, false); %solve

%% Mixed traffic strict eco-ACC
c_strict = DP_1D_timepenalty_E(Route,in_strict, bounds_1D, sim); %find time penalty
[~, Res_strict] = DP_1D(Route, in_strict, bounds_1D, sim, c_strict, false); %solve

%% Dedicated lane
c_dl = DP_1D_timepenalty(Route_dl,in, bounds_1D, sim); %find time penalty
[~, Res_dl] = DP_1D(Route_dl, in, bounds_1D, sim, c_dl, false); %solve

%% Mixed Traffic relaxed eco-ACC with Signal Phase and Timing information
Res_relaxed_SPaT = DP_2D(Route, in, bounds_2D, sim); %solve

%% Mixed traffic strict eco-ACC with Signal Phase and Timing information
Res_strict_SPaT = DP_2D(Route, in_strict, bounds_2D, sim); %solve

%% Dedicated lane with Signal Phase and Timing information
Res_dl_SPaT = DP_2D(Route_dl, in, bounds_2D, sim); %solve

%% Calculate energy consumption
E_measured      = Econs(Route.t_measured,Route.v_measured,in);
E_strict        = Econs(Res_strict.t,Res_strict.v,in);
E_relaxed       = Econs(Res_relaxed.t,Res_relaxed.v,in);
E_dl            = Econs(Res_dl.t,Res_dl.v,in);
E_strict_SPaT    = Econs(Res_strict_SPaT.t,Res_strict_SPaT.v,in);
E_relaxed_SPaT    = Econs(Res_relaxed_SPaT.t,Res_relaxed_SPaT.v,in);
E_dl_SPaT       = Econs(Res_dl_SPaT.t,Res_dl_SPaT.v,in);

%% Calculate Intervehicle distances and eco-driving percentages
[ds_max_loose, eco_share_loose] = ds_plotter(Res_relaxed,Route,in,false);
[ds_max_tight, eco_share_tight] = ds_plotter(Res_strict,Route,in_strict,false);
[ds_max_loose_SPaT, eco_share_loose_SPaT] = ds_plotter(Res_relaxed_SPaT,Route,in,false);
[ds_max_tight_SPaT, eco_share_tight_SPaT] = ds_plotter(Res_strict_SPaT,Route,in_strict,false);

%% Save results
filename = ['Results/' Route.name '_' in.powertrain '_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '.mat'];
save(filename)
