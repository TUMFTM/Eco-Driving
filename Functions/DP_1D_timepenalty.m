function c_time = DP_1D_timepenalty(Route, in, bounds, sim)
%Find the time penalty that minimizes the difference between the target
%time and trip duration

%% Optimize time penalty in DP-1D
f = @(c_time) DP_1D(Route, in, bounds, sim, c_time, false); % Transform into a function where c_time is the only variable
options = optimset('Display','iter','TolX',1e-5); %optimization options
c_time = fzero(f,[0 in.cmax],options);

end