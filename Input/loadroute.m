function Route = loadroute(Routename)

switch Routename
    case 'Route91'
        load('Route91.mat','Route');
    case 'Route91_first3stops'
        load('Route91_first3stops.mat','Route');
    case 'Route185'
        load('Route185.mat','Route'); 
    case 'Custom'
        Route.name = 'test';
        Route.s_vlim = [0 300];
        Route.v_vlim = [30 30]/3.6;
        Route.s_tl = [];
        Route.t_firstgreen = [];
        Route.t_green = [];
        Route.t_red = [];
        Route.s_stop = [0 300];
        Route.t_stop = [0 0];
        Route.s_measured = [];
        Route.v_measured = [];
        Route.t_measured = [];
        Route.s_corner = [];
        Route.v_corner = []/3.6;
        Route.tmax = 100;
    otherwise
        error('Route not available')
end

end