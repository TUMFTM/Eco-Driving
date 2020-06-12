% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Definitions (optional): 
%{
SPaT: Signal Phase and Timing. Vehicle is aware of upcoming traffic light
strict: adaptive cruise control has a maximum and a minimum intervehicle
distance
relaxed: adaptive cruise control only has a minimum intervehicle distance 
%}
% ------------
% Description: 
%{
This script computes the optimal speed profile for a bus, taking speed
limits, turns, stops, traffic lights and a leading vehicle into account.
The code compares 6 different scenarios:

                     | \wo SPaT  | with SPaT
Strict car following |    x      |     x
Loose care following |    x      |     x
Dedicated lane       |    x      |     x

%}
% ------------
% Input:    speed limits, corners, stops and traffic lights of the route
% ------------
% Output:   optimized speed profiles
% ------------

close all
clearvars
clc
addpath(genpath(pwd))
global strict

%% Input
Route = loadroute('Route91_first3stops'); % load boundary conditions
constants

%Dedicated lane
Route_dl = Route; %Create special case for dedicated lane
Route_dl.s_measured = []; %by removing the leading vehicle

%% Preprocessing
bounds_1D = DP_1D_bounds(Route); %bounds and traffic lights for 1D state space
bounds_2D = DP_2D_bounds(Route, bounds_1D, false); %bounds and traffic lights for 2D state space
sim = DP_veh_sim(bounds_1D); %vehicle simulation

%% Mixed Traffic relaxed eco-ACC
strict = false;% Strict or loos vehicle following
c_relaxed = DP_1D_timepenalty(Route, bounds_1D, sim); %find time penalty
[~, Res_relaxed] = DP_1D(Route, bounds_1D, sim, c_relaxed, false); %solve

%% Mixed traffic strict eco-ACC
strict = true;% Strict or loos vehicle following
c_strict = DP_1D_timepenalty_E(Route, bounds_1D, sim); %find time penalty
[~, Res_strict] = DP_1D(Route, bounds_1D, sim, c_strict, false); %solve

%% Dedicated lane
c_dl = DP_1D_timepenalty(Route_dl, bounds_1D, sim); %find time penalty
[~, Res_dl] = DP_1D(Route_dl, bounds_1D, sim, c_dl, false); %solve

%% Mixed Traffic relaxed eco-ACC with Signal Phase and Timing information
strict = false;% Strict or loos vehicle following
Res_relaxed_SPaT = DP_2D(Route, bounds_2D, sim); %solve

%% Mixed traffic strict eco-ACC with Signal Phase and Timing information
strict = true;% Strict or loos vehicle following
Res_strict_SPaT = DP_2D(Route, bounds_2D, sim); %solve

%% Dedicated lane with Signal Phase and Timing information
Res_dl_SPaT = DP_2D(Route_dl, bounds_2D, sim); %solve

%% Calculate energy consumption
E_measured      = Econs(Route.t_measured,Route.v_measured);
E_strict        = Econs(Res_strict.t,Res_strict.v);
E_relaxed       = Econs(Res_relaxed.t,Res_relaxed.v);
E_dl            = Econs(Res_dl.t,Res_dl.v);
E_strict_SPaT    = Econs(Res_strict_SPaT.t,Res_strict_SPaT.v);
E_relaxed_SPaT    = Econs(Res_relaxed_SPaT.t,Res_relaxed_SPaT.v);
E_dl_SPaT       = Econs(Res_dl_SPaT.t,Res_dl_SPaT.v);

%% Calculate Intervehicle distances and eco-driving percentages
[ds_max_loose, eco_share_loose] = ds_plotter(Res_relaxed,Route,false);
[ds_max_tight, eco_share_tight] = ds_plotter(Res_strict,Route,false);
[ds_max_loose_SPaT, eco_share_loose_SPaT] = ds_plotter(Res_relaxed_SPaT,Route,false);
[ds_max_tight_SPaT, eco_share_tight_SPaT] = ds_plotter(Res_strict_SPaT,Route,false);

%% Save results
filename = ['Results/' Route.name '_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '.mat'];
save(filename)
