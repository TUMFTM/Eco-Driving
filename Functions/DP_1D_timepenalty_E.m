function c_time = DP_1D_timepenalty_E(Route, in, bounds, sim)
%Find the time penalty that minimizes energy consumption.
%The strict eco-ACC can't minimize the difference to the target time, as in
%the relaxed eco-ACC and dedicated lane use cases, since the maximum inter-vehicle distance
%ensures that the vehicle always arrives close to the target time. 

f = @(c_time) Emin(Route, in, bounds, sim, c_time); % Transform into a function where c_time is the only variable
options = optimset('Display','iter','TolX',10,'TolFun',1);
c_time = fminsearch(f,1e5,options);

end

function E = Emin(Route, in, bounds, sim, c_time)
[~, Res_mixed] = DP_1D(Route, in, bounds, sim, c_time, false);
E = Econs(Res_mixed.t,Res_mixed.v,in);

end