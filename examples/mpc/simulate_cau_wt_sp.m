% Compare two cautious drivers with ego car supervised or not supervised.
% better visual
% solve dual delta
clear all;close all;clc;
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
CIS = CIS_cau;
preXU = preXU_cau;
dim = 4;
x0 = [30 0.1 22 25]';
if ~containsPolyUnion(CIS,x0(1:dim))
    error("bad initial point!")
end

xA1 = x0;
X_list = x0;
U_list = [];
US_list = [];
T_list = [0];
% function simulate_intention(x0)
con = constants_tri;
time_horizon = 15;
plot_stuff = 1;


UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

% initial conditions
vEgoA1 = x0(1); yEgoA1 = x0(2); hA1 = x0(3); vLeadA1 = x0(4);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
XA1 = zeros(4,time_horizon/con.dt+1);

% simulate
fig = figure('position',[100 100 1200 640]);
counter = 1;

for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoA1 = -95 + sum(XA1(1,:))*con.dt;
    % keep history of the states
    XA1(:,index) = xA1;

    % put your controller here
%     u_c1 = mpc_tailgate(xA1(1:3),xA1(4),con,t);
    [u_c1,u_mpc,~,U_f1] = mpc_supervisory(xA1,CIS, preXU, con, t);
    if abs(xA1(3)) >= con.h_reaction
       U_f1 = get_input(preXU_bnd,xA1,3);
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
        plot_road(xEgoA1,t);
        plot_car1(xEgoA1,xA1);
        % plot input region
        subplot(212);
        title = "input of ego1";
        plot_safe_input(U_f1,title,u_mpc, u);
        drawnow;
        hold off;
    end
    w = [wx(index),wy(index),wL(index)];
    xA1 = update_agg(xA1,u,w,UnSafe,con);
    
    % record state history
    X_list = [X_list, xA1];
    U_list = [U_list,u];
    US_list = [US_list,U_f1];
    T_list = [T_list, t];
%     print(fig,['./pic/frame',num2str(counter-1)],'-dpng');
end
%%
save traj_cau.mat X_list U_list T_list US_list
