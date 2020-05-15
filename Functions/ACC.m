function [s_new, t_new, v_new] = ACC(s1, t1, v1, s2, t2, v2, Route, in)

diff_smin = s_intervehicle(s1,t1,v1,v2(1),Route,in,'min');
diff_smax = s_intervehicle(s1,t1,v1,v2(1),Route,in,'max');

%Adjust speed if the limits are violated
if diff_smin<0 %Lower distance limit is violated
    vmax = sqrt(v1^2+2*in.amax*in.ds);    
    f_smin = @(v2) s_intervehicle(s1,t1,v1,v2,Route,in,'min');
    
    try
        v_new = fzero(f_smin,[0 vmax]);
    catch %if a speed lower than zero is required
        v_new = 0;
    end   
    
    %Add extra point for stop if necessary
    if v_new == 0
        t_pass = Route.t_measured(find(Route.s_measured>s1+3,1));
        if t_pass>t1+2*in.ds/v1
            s_new = [s1 s1] + in.ds;
            t_new = [t1+2*in.ds/v1 t_pass];
            v_new = [0 0];
        else
            s_new = s1 + in.ds;
            t_new = t1+2*in.ds/v1;
            v_new = 0;
        end
    else
        s_new = s1+in.ds;
        t_new = t1+2*in.ds./(v1+v_new);
    end
elseif diff_smax>0 %Upper distance limit is violated
    f_smax = @(v2) s_intervehicle(s1,t1,v1,v2,Route,in,'max');
    vmax = sqrt(v1^2+2*in.amax*in.ds);
    vmin = sqrt(v1^2-2*in.amax*in.ds);
    try
        v_new = min([max(Route.v_vlim) fzero(f_smax,[vmin vmax])]);       
    catch
        v_new = min([max(Route.v_vlim) sqrt(v1.^2+2*in.amax*in.ds)]);
    end
    s_new = s1+in.ds;
    t_new = t1+2*in.ds./(v1+v_new);
else %Follow vector field
    s_new = s2;
    t_new = t2;
    v_new = v2;
end

end