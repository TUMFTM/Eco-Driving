function bounds = DP_2D_bounds(Route, bounds, plottrue)
% Designed by: Olaf Teichert (FTM, Technical University of Munich)
%-------------
% Created on: 2020-02-24
% ------------
% Version: Matlab2020a
%-------------
% Description: calculates the time bound based on the speed bound, traffic
% light phases and stop durations
% ------------
% Input:    - Route: struct containing the route characteristics
%           - in: struct containing all global constants
%           - bounds: struct containing the 1D boundary conditions
% ------------
% Output:   - bounds: struct containing the 1D boundary conditions and the 
% 2D boundary conditions
% ------------ 

    global ds t_res

    %% Lower time bound
    v_avg = movmean(bounds.v,2); %Calculate average speed for time bound calculation
    t_lower = [0 cumsum(ds./v_avg(2:end))]; %Calculate lower time bound
    
    %% Adjust time bound for stops
    for i = 1:length(Route.s_stop) %Adjust for stops   
        I_stop = find(bounds.s<Route.s_stop(i),1,'last');
        t_lower(I_stop:end) = t_lower(I_stop:end)+Route.t_stop(i);
    end
    
    %% Upper time bound
    t_upper = t_lower+Route.tmax-t_lower(end);
    
    %% Adjust time bounds for traffic lights
    
    %Adjust lower time bound    
    for i = 1:length(Route.s_tl)
        I_tl = find(bounds.s>=Route.s_tl(i),1);
        
        t_tl_bound =  t_lower(I_tl); %time at which the timebound reaches the traffic light
        t_tl_bound_red_phases = t_tl_bound-bounds.t_redswitches{i}; %time difference to start of the red phases
        t_tl_bound_lastred = min(t_tl_bound_red_phases(t_tl_bound_red_phases>0)); %time at which the last red phase started           
        if t_tl_bound_lastred<Route.t_red(i) %If the time bound hits a red phase, shift it up
            t_lower(I_tl:end) = t_lower(I_tl:end)+Route.t_red(i)-t_tl_bound_lastred;
        end
    end

    %Adjust upper time bound
    for i = length(Route.s_tl):-1:1
        I_tl = find(bounds.s>=Route.s_tl(i),1);
        
        t_tl_bound =  t_upper(I_tl); %time at which the timebound reaches the traffic light
        t_tl_bound_red_phases = t_tl_bound-bounds.t_redswitches{i}; %time difference to start of the red phases
        t_tl_bound_lastred = min(t_tl_bound_red_phases(t_tl_bound_red_phases>0)); %time at which the last red phase started           
        if t_tl_bound_lastred<Route.t_red(i) %If the time bound hits a red phase, shift it down
            t_upper(1:I_tl) = t_upper(1:I_tl)-t_tl_bound_lastred;
        end
    end
    
    %% Check feasibility
    if or(t_upper(1)<0,t_lower(end)>Route.tmax)
        bounds = [];
        disp('Route cannot be completed in available time')
        return
    end
    
    %% Create time space
    bounds.t_space = zeros(length(bounds.s),t_res);
    for i = 1:length(bounds.s)
        bounds.t_space(i,:) = sort([linspace(t_lower(i),t_upper(i)-10,t_res-10) t_upper(i)-9:t_upper(i)]); %Extra points are added near upper boundary to avoid errors
    end    
 
    %% Plot
    if plottrue
        figure
        hold on
        plot(Route.s_measured,Route.v_measured*3.6)
        plot(bounds.s,bounds.v*3.6,'k--')
        xlabel('Distance in meters')
        ylabel('Speed in km/h')
        legend({'Measurement','Speed boundary'},'Location','NorthWest')
        xlim([0 bounds.s(end)])
        
        figure
        hold on
        H_m = plot(Route.s_measured,Route.t_measured);
        H_b = plot(bounds.s,t_lower,'k--');
        plot(bounds.s,t_upper,'k--')
        H_tl = [];
        for i = 1:length(Route.s_tl)
            H_tl = plot([Route.s_tl(i); Route.s_tl(i)],[bounds.t_redswitches{i}; bounds.t_greenswitches{i}],'r');
        end
        if isempty(H_tl)
            legend([H_m, H_b],{'Measurement','time boundaries'},'Location','Best')
        else
            legend([H_m, H_b, H_tl(1)],{'Measurement','Time boundaries','Traffic lights'},'Location','Best')
        end
        ylim([0 Route.tmax])
        xlim([0 bounds.s(end)])
        xlabel('Distance in meters')
        ylabel('Time in seconds')
    end
end