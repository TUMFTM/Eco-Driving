function [ds_max, eco_share] = ds_plotter(Res,Route,in,plottrue)

ds = interp1(Route.t_measured,Route.s_measured,Res.t)-Res.s;
smin = zeros(1,length(Res.t));
smax = zeros(1,length(Res.t));
smax(1) = in.lcar;
for i = 2:length(Res.t)
    [~, ~, smin(i), smax(i)] = s_intervehicle(Res.s(i-1),Res.t(i-1),Res.v(i-1),Res.v(i),Route,in,'min');
end

ds_max = max(ds);
if in.tight
    I_eco = and(ds>smin*1.01,ds<smax*0.99);
else
    I_eco = ds>smin*1.01;
end
eco_share = sum(I_eco)/length(ds);

if plottrue
    figure('Units','Points','Position',[300 300 225 130])
    set(gca,'FontSize',8,'FontName','Times New Roman')
    hold on
    plot(Res.s,smin+in.lzero,'r--')
    if in.tight
        plot(Res.s,smax+in.lzero,'r--')
    end
    plot(Res.s,ds+in.lzero,'k')
%     plot(Res.s(I_eco),ds(I_eco)+in.lzero,'*')
    xlabel('Distance in meters')
    ylabel('Inter-vehicle distance in meters')
    xlim([0 1239])
    ylim([0 90])
end

end