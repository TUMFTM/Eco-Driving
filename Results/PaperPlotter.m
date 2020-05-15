% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: Plots all papers in the paper
% ------------
% Definitions (optional): first column as ... second column as ...
% ------------
% Input:    - Saved results structs
% ------------
% Output:   - Paper plots
% ------------

clearvars
close all
clc

load('Route91_first3stops_Combustion_20200515_175734.mat')

%% Boundary conditions
DP_2D_bounds(Route, In, bounds_1D, true);
set(gcf,'Units','Points','Position',[300 300 246 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% Vector Fields no tl
Route_dl_notl = Route_dl;
Route_dl_notl.s_tl = [];
In.v_res = 500;
bounds_1D_notl = DP_1D_bounds(Route_dl_notl, In); 
sim_notl = DP_veh_sim(In, bounds_1D_notl); %vehicle simulation
c_dl_notl = DP_1D_timepenalty(Route_dl_notl,In, bounds_1D_notl, sim_notl); %find time penalty
DP_1D(Route_dl_notl, In, bounds_1D_notl, sim_notl, c_dl_notl, true); %solve
xlim([0 371])
set(gcf,'Units','Points','Position',[300 300 246 160])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% Vector field with tl
In.v_res = 100;
sim = DP_veh_sim(In, bounds_1D); %vehicle simulation
DP_1D(Route_dl, In, bounds_1D, sim, c_dl, true); %With traffic lights
xlim([0 371])
set(gcf,'Units','Points','Position',[300 300 246 160])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% Traffic light approach strategy
s_plot = -100:0;
Froll = In.m*In.g*In.fr; %Rolling resistance [N]

vdec = sqrt(-2*In.amax*s_plot); %max. deceleration speed
vcoast = sqrt(-2*s_plot*Froll./(In.m-s_plot*In.rho*In.cw*In.A)); %Coasting speed

figure
hold on
plot(s_plot,vdec*3.6,'--')
plot(s_plot,vcoast*3.6)
ylim([0 60])
xlabel('Distance to traffic light in meters')
ylabel('Speed in km/h')
legend({'Deceleration with a_{min}','Coasting'},'Location','Best','Orientation','Horizontal')
text(-49.5,1.5*3.6,'I','FontSize',8,'FontName','Times New Roman')
text(-50,6.5*3.6,'II','FontSize',8,'FontName','Times New Roman')
text(-50.5,12*3.6,'III','FontSize',8,'FontName','Times New Roman')

set(gcf,'Units','Points','Position',[300 300 246 130])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% strict
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_strict.s,Res_strict.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% loose
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_relaxed.s,Res_relaxed.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% Dl
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_dl.s,Res_dl.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% tight SPaT
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_strict_SPaT.s,Res_strict_SPaT.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% loose SPaT
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_relaxed_SPaT.s,Res_relaxed_SPaT.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% Dedicated lane SPaT
figure
hold on
xlabel('Distance in meters')
ylabel('Speed in km/h')
plot(Route.s_measured,Route.v_measured*3.6,'--')
plot(Res_dl_SPaT.s,Res_dl_SPaT.v*3.6)
xlim([0 1239])
ylim([0 50])
set(gcf,'Units','Points','Position',[300 300 150 120])
set(gca,'FontSize',8,'FontName','Times New Roman')

%% ds curves
ds_plotter(Res_strict,Route,in_strict,true);
ds_plotter(Res_relaxed, Route,In,true);
ds_plotter(Res_strict_SPaT,Route,in_strict,true);
ds_plotter(Res_relaxed_SPaT,Route,In,true);

%% Comparison
clearvars
load('Route91_Combustion_20200312_144552.mat')

E_plot = [E_strict E_relaxed E_dl E_strict_SPaT E_relaxed_SPaT E_dl_SPaT];

figure('Units','Points','Position',[300 300 246 200])
set(gca,'FontSize',8,'FontName','Times New Roman')
hold on
plot([0.5 6.5],[E_measured E_measured],'r--')
text(0.2 + 7.4/2,E_measured,'Measurement benchmark','HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',8,'FontName','Times New Roman')
bar(E_plot)
ylabel('Energy consumption in kWh/km')
xlabel('Scenario')
xticks(1:6)
xticklabels({'I','II','III','IV','V','VI'})

x_delta = 0.1;
y_delta = 0.2;
h_a = get(gca,'Position');
for i = 1:length(E_plot)
    x = h_a(1)+(i+0.2)*h_a(3)/7.4;
    y1 = h_a(2)+h_a(4)*E_measured/3.5;
    y2 = h_a(2)+h_a(4)*E_plot(i)/3.5;
    annotation('textarrow',[x x],[y1 y2])
    text(i+x_delta,E_measured-y_delta,['-' num2str(round(100*(E_measured-E_plot(i))/E_measured)) '%'],'FontSize',8,'FontName','Times New Roman')
end