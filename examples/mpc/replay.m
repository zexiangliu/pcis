function replay(exp1,exp2)
con = constants_tri;
pic_path = 'pic/';
mkdir(pic_path);
tstep = length(exp1.T_list);
fig = figure('position',[100 100 1200 640]);

for k = 1:tstep
    t = exp1.T_list(k);
    xEgoA1 = -95 + sum(exp1.X_list(1,1:k-1))*con.dt;
    xEgoA2 = -95 + sum(exp2.X_list(1,1:k-1))*con.dt;
    xA1 = exp1.X_list(:,k);
    xA2 = exp2.X_list(:,k);
    u1 = exp1.U_list(:,k);
    u2 = exp2.U_list(:,k);
    U_f1 = exp1.US_list(k);
    U_f2 = exp2.US_list(k);
    if t <= exp1.t_detection
        int_est(1) = "unknown";
    else
        int_est(1) = exp1.lead_dyn;
    end
    
    if t < exp2.t_detection
        int_est(2) = "unknown";
    else
        int_est(2) = exp1.lead_dyn;
    end
    %figure;
    clf; 
    subplot(311)
    hold on    
    plot_road(xEgoA1,t);
    plot_car1(xEgoA1,xA1);
    plot_car2(xEgoA2,xA2);
    subplot(312);
    hold on
    title = "input of ego1"+" (current intention estimate: "+int_est(1)+")";
    if exp1.sp_on    
        u_mpc1 = exp1.UMPC_list(:,k);
        plot_safe_input(U_f1,title,u_mpc1, u1);
    elseif exp2.sp_on
        plot_safe_input(U_f2,title,u1);
    else
        plot_safe_input(U_f1,title,u1);
    end
    subplot(313);
    hold on
    title = "input of ego2"+" (current intention estimate: "+int_est(2)+")";
    if exp2.sp_on
        u_mpc2 = exp2.UMPC_list(:,k);
        plot_safe_input(U_f2,title,u_mpc2, u2);
    elseif exp1.sp_on
        plot_safe_input(U_f1,title,u2);
    else
        plot_safe_input(U_f2,title,u2);
    end
    drawnow;
    hold off
    print(fig,[pic_path,'frame',num2str(k-1)],'-dpng');
end
end