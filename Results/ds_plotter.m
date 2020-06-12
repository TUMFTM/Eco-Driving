function [sdiff_max, eco_share] = ds_plotter(Res,Route,plottrue)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: plots intervehicle distance curves
% ------------
% Input:    - Res: struct containing optimized speed profile
%           - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - plottrue: flag for plotting optimal speed profile as boolean
% ------------
% Output:   - ds_max: maximum intervehicle distance
%           - eco-share: share of distance in which the vehicle could
%           operate in eco-driving mode without intervehicle distance
%           constraints
% ------------

global lzero lcar strict

sdiff = interp1(Route.t_measured,Route.s_measured,Res.t)-Res.s;
smin = zeros(1,length(Res.t));
smax = zeros(1,length(Res.t));
smax(1) = lcar;
for i = 2:length(Res.t)
    [~, ~, smin(i), smax(i)] = s_intervehicle(Res.s(i-1),Res.t(i-1),Res.v(i-1),Res.v(i),Route,'min');
end

sdiff_max = max(sdiff);
if strict
    I_eco = and(sdiff>smin*1.01,sdiff<smax*0.99);
else
    I_eco = sdiff>smin*1.01;
end
eco_share = sum(I_eco)/length(sdiff);

if plottrue
    figure('Units','Points','Position',[300 300 225 130])
    set(gca,'FontSize',8,'FontName','Times New Roman')
    hold on
    plot(Res.s,smin+lzero,'r--')
    if strict
        plot(Res.s,smax+lzero,'r--')
    end
    plot(Res.s,sdiff+lzero,'k')
    xlabel('Distance in meters')
    ylabel('Inter-vehicle distance in meters')
    xlim([0 1239])
    ylim([0 90])
end

end