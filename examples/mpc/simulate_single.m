% Compare two cautious drivers with ego car supervised or not supervised.
% better visual
% solve dual delta
clear all;close all;clc;
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
load CIS_bnd.mat

Iv = 'bnd';
Iv_CIS = CIS_bnd.copy();
Iv_preXU = preXU_bnd.copy();
dim = Iv_CIS.Set(1).Dim;
x0 = [30 0.1 50 25]';
if ~containsPolyUnion(Iv_CIS,x0(1:dim))
    error("bad initial point!")
end

% function simulate_intention(x0)
con = constants_tri;
time_horizon = 15;
plot_stuff = 1;
is_int_on = 1; % is intention on
is_sp_on = 0; % is supervisor on
is_print = 0; % print the fig or not
pic_path = './pic/';
lead_int = "agg";
mkdir(pic_path);

UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

% initial conditions
vEgoA1 = x0(1); yEgoA1 = x0(2); hA1 = x0(3); vLeadA1 = x0(4);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
xA1 = x0;
X_list = [x0,zeros(4,time_horizon/con.dt)];
U_list = zeros(2,time_horizon/con.dt);
US_list = [];
T_list = zeros(1,time_horizon/con.dt+1);

% simulate
if plot_stuff
    fig = figure('position',[100 100 1200 640]);
end
counter = 1;

for t = 0:con.dt:time_horizon
    t
    counter = counter+1;
    % visualize
    xEgoA1 = -95 + sum(X_list(1,1:end-1))*con.dt;

    % put your controller here
    if is_sp_on
        [u_c1, u_mpc, ~, U_f1] = mpc_supervisory(xA1,Iv_CIS, Iv_preXU, con, t);
    else    
        u_c1 = mpc_tailgate(xA1(1:3),xA1(4),con,t);    
        U_f1 = get_input(Iv_preXU,xA1,dim);
    end
    u_c1 = u_c1(:,1);
%     u_c1 = mpc_supervisory(xA1,CIS_bnd, preXU_bnd, con);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    u = [aEgoA1;vyEgoA1];
    if plot_stuff
        %figure;
        clf; 
        subplot(211)
        hold on    
        plot_road(xEgoA1,t);
        plot_car1(xEgoA1,xA1);

        subplot(212);
        hold on
        title = "input of ego1";
        if is_sp_on
            plot_safe_input(U_f1,title,u_mpc, u);
        else
            plot_safe_input(U_f1,title,u);
        end
        drawnow;
        hold off
    end
    % make sure the car respects velocity and acceleration bounds
    w = [wx(counter-1),wy(counter-1),wL(counter-1)];
    if Iv == "bnd" && is_int_on
        [xA1, Iv] = update_dyn(xA1,u,w,UnSafe,con,lead_int);
        if strcmp(Iv, 'ann')
            Iv_CIS = CIS_ann.copy();
            Iv_preXU = preXU_ann.copy();
            dim = Iv_CIS.Set(1).Dim;
            t_detection = t;
        elseif strcmp(Iv, 'cau')
            Iv_CIS = CIS_cau.copy();
            Iv_preXU = preXU_cau.copy();
            dim = Iv_CIS.Set(1).Dim;
            t_detection = t;
        end
    else
        xA1 = update_dyn(xA1,u,w,UnSafe,con,lead_int);
    end

    X_list(:,counter) = xA1;
    U_list(:,counter) = u_c1;
    T_list(counter) = t;
    US_list = [US_list,U_f1];
    if plot_stuff && is_print
        print(fig,[pic_path,'frame',num2str(counter-1)],'-dpng');
    end
end
save(filename,X_list,U_list,US_list,T_list,t_detection);
