function [traj1,traj2] = simulate_compare(varargin)
% Simulate the takeover scenario with two lead car and two ego car.

p = inputParser;
p.addParameter('time_horizon',15);
p.addParameter('plot_stuff',1);
p.addParameter('is_sp_on',[0 1]);
p.addParameter('is_print',0);
p.addParameter('pic_path','./pic/');
p.addParameter('lead_dyn',["agg","agg"]);
p.addParameter('Iv',["bnd","agg"]);
p.addParameter('sp_with_int',[0 0]);
p.addParameter("x0", [30 0.1 22 25]');
p.addParameter("mpc","tailgate")
p.parse(varargin{:});


time_horizon = p.Results.time_horizon;
plot_stuff = p.Results.plot_stuff;
is_sp_on = p.Results.is_sp_on; % is supervisor on
is_print = p.Results.is_print; % print the fig or not
pic_path = p.Results.pic_path;
lead_dyn = p.Results.lead_dyn;
sp_with_int = p.Results.sp_with_int;
mpc = p.Results.mpc;
Iv = p.Results.Iv;
x0 = p.Results.x0;
if is_print
    mkdir(pic_path);
end

% load invariant set
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
load CIS_bnd.mat


for i = 1:2
    if Iv(i) == "bnd"
        Iv_CIS(i) = CIS_bnd.copy();
        Iv_preXU(i) = preXU_bnd.copy();    
    elseif lead_dyn(i) == "agg"
        Iv_CIS(i) = CIS_ann.copy();
        Iv_preXU(i) = preXU_ann.copy();
    elseif lead_dyn(i) == "cau"
        Iv_CIS(i) = CIS_cau.copy();
        Iv_preXU(i) = preXU_cau.copy();
    end
    dim(i) = Iv_CIS(i).Set(1).Dim;
end

if ~containsPolyUnion(Iv_CIS(1),x0(1:dim))
    error("bad initial point!")
end

UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
xA1 = x0;
xA2 = x0;
X1_list = [x0,zeros(4,time_horizon/con.dt)];
U1_list = zeros(2,time_horizon/con.dt);
US1_list = [];
T1_list = zeros(1,time_horizon/con.dt+1);
X2_list = [x0,zeros(4,time_horizon/con.dt)];
U2_list = zeros(2,time_horizon/con.dt);
US2_list = [];
T2_list = zeros(1,time_horizon/con.dt+1);
if is_sp_on(1)
    UMPC1_list = U1_list;
end
if is_sp_on(2)
    UMPC2_list = U2_list;
end

% simulate
if plot_stuff
    fig = figure('position',[100 100 1200 640]);
end
counter = 1;
t_detection = [inf, inf];
int_est = ["bnd","bnd"];
for t = 0:con.dt:time_horizon
    t
    counter = counter+1;
    % visualize
    xEgoA1 = -95 + sum(X1_list(1,1:end-1))*con.dt;
    xEgoA2 = -95 + sum(X2_list(1,1:end-1))*con.dt;

    % put your controller here
    if is_sp_on(1)
        if mpc == "tailgate"
            [u_c1, u_mpc1, ~, U_f1] = mpc_supervisory(xA1,Iv_CIS(1), Iv_preXU(1), con, t);
        elseif mpc == "takeover"
            [u_c1, u_mpc1, ~, U_f1] = mpc_supervisory(xA1,Iv_CIS(1), Iv_preXU(1), con);
        end
    else    
        if mpc == "tailgate"
            u_c1 = mpc_tailgate(xA1(1:3),xA1(4),con,t);    
        elseif mpc == "takeover"
            u_c1 = mpc_simple(xA1(1:3),xA1(4),con);    
        end
        U_f1 = get_input(Iv_preXU(1),xA1,dim(1));
    end

    if abs(xA1(3)) >= con.h_reaction && Iv(1) ~= "bnd"
       U_f1 = get_input(preXU_bnd,xA1,3);
    end
    
    if is_sp_on(2)
        if mpc == "tailgate"
            [u_c2, u_mpc2, ~, U_f2] = mpc_supervisory(xA2,Iv_CIS(2), Iv_preXU(2), con, t);
        else
            [u_c2, u_mpc2, ~, U_f2] = mpc_supervisory(xA2,Iv_CIS(2), Iv_preXU(2), con);
        end
    else    
        if mpc == "tailgate"
            u_c2 = mpc_tailgate(xA2(1:3),xA2(4),con,t);    
        elseif mpc == "takeover"
            u_c2 = mpc_simple(xA2(1:3),xA2(4),con);    
        end   
        U_f2 = get_input(Iv_preXU(2),xA2,dim(2));
    end

    if abs(xA2(3)) >= con.h_reaction && Iv(2) ~= "bnd"
       U_f2 = get_input(preXU_bnd,xA2,3);
    end

    u_c1 = u_c1(:,1);
    u_c2 = u_c2(:,1);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    u1 = [aEgoA1;vyEgoA1];
    aEgoA2 = u_c2(1,1); 
    vyEgoA2 = u_c2(2,1);
    aEgoA2 =  max(min(aEgoA2, con.umax_ACC), con.umin_ACC);
    vyEgoA2 =  max(min(vyEgoA2, con.umax_LK), con.umin_LK);
    u2 = [aEgoA2;vyEgoA2];
    if plot_stuff
        %figure;
        clf; 
        subplot(311)
        hold on    
        plot_road(xEgoA1,t);
        plot_car1(xEgoA1,xA1);
        plot_car2(xEgoA2,xA2);
        subplot(312);
        hold on
        title = "input of ego1"+" (current intention: "+int_est(1)+")";
        if is_sp_on(1)
            plot_safe_input(U_f1,title,u_mpc1, u1);
        else
            plot_safe_input(U_f1,title,u1);
        end
        subplot(313);
        hold on
        title = "input of ego2"+" (current intention: "+int_est(2)+")";
        if is_sp_on(2)
            plot_safe_input(U_f2,title,u_mpc2, u2);
        else
            plot_safe_input(U_f2,title,u2);
        end
        drawnow;
        hold off
    end
    % make sure the car respects velocity and acceleration bounds
    w = [wx(counter-1),wy(counter-1),wL(counter-1)];
    if int_est(1) == "bnd"
        [xA1, int_est(1)] = update_dyn(xA1,u1,w,UnSafe,con,lead_dyn(1));
        if sp_with_int(1)
            if strcmp(int_est(1), 'ann')
                Iv(1) = int_est(1);
                Iv_CIS(1) = CIS_ann.copy();
                Iv_preXU(1) = preXU_ann.copy();
                dim(1) = Iv_CIS(1).Set(1).Dim;
            elseif strcmp(int_est(1), 'cau')
                Iv(1) = int_est(1);
                Iv_CIS(1) = CIS_cau.copy();
                Iv_preXU(1) = preXU_cau.copy();
                dim(1) = Iv_CIS(1).Set(1).Dim;
            end
        end
        t_detection(1) = t;
    else
        xA1 = update_dyn(xA1,u1,w,UnSafe,con,lead_dyn(1));
    end
    if int_est(2) == "bnd"
        [xA2, int_est(2)] = update_dyn(xA2,u2,w,UnSafe,con,lead_dyn(2));
        if sp_with_int(2)
            if strcmp(int_est(2), 'ann')
                Iv(2) = int_est(2);
                Iv_CIS(2) = CIS_ann.copy();
                Iv_preXU(2) = preXU_ann.copy();
                dim(2) = Iv_CIS(2).Set(1).Dim;
            elseif strcmp(int_est(2), 'cau')
                Iv(2) = int_est(2);
                Iv_CIS(2) = CIS_cau.copy();
                Iv_preXU(2) = preXU_cau.copy();
                dim(2) = Iv_CIS(2).Set(2).Dim;
            end
        end
        t_detection(2) = t;
    else
        xA2 = update_dyn(xA2,u2,w,UnSafe,con,lead_dyn(2));
    end

    X1_list(:,counter) = xA1;
    U1_list(:,counter) = u_c1;
    T1_list(counter) = t;
    US1_list = [US1_list,U_f1];
    X2_list(:,counter) = xA2;
    U2_list(:,counter) = u_c2;
    T2_list(counter) = t;
    US2_list = [US2_list,U_f2];
    if is_sp_on(1)
        UMPC1_list(:,counter) = u_mpc1;
    end
    if is_sp_on(2)
        UMPC2_list(:,counter) = u_mpc2;
    end
    if plot_stuff && is_print
        print(fig,[pic_path,'frame',num2str(counter-1)],'-dpng');
    end
end

traj1.Iv = Iv(1);
traj1.lead_dyn = lead_dyn(1);
traj1.sp_on = is_sp_on(1);
traj1.sp_with_int = sp_with_int(1);
traj1.X_list = X1_list;
traj1.U_list = U1_list;
traj1.US_list = US1_list;
traj1.T_list = T1_list;
traj1.t_detection = t_detection(1);

if is_sp_on(1)
    traj1.UMPC_list = UMPC1_list;
end

traj2.Iv = Iv(2);
traj2.lead_dyn = lead_dyn(2);
traj2.sp_on = is_sp_on(2);
traj2.sp_with_int = sp_with_int(2);
traj2.X_list = X2_list;
traj2.U_list = U2_list;
traj2.US_list = US2_list;
traj2.T_list = T2_list;
traj2.t_detection = t_detection(2);

if is_sp_on(2)
    traj2.UMPC_list = UMPC2_list;
end
