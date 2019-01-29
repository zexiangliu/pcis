% Compare two cautious drivers with ego car supervised or not supervised.
% better visual
% solve dual delta
clear all;close all;clc;
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
CIS = CIS_cau;
pre_XU = preXU_cau;
dim = 4;
x0 = [30 0.1 22 25]';
if ~containsPolyUnion(CIS,x0(1:dim))
    error("bad initial point!")
end

xA1 = x0;
X_list = x0;
U_list = [];
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
msiz = 200; % marker size
malpha = 0.5; % marker alpha
fsiz = 12;
counter = 1;

for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoA1 = -95 + sum(XA1(1,:))*con.dt;
    if plot_stuff
        %figure;
        clf; 
        subplot(311)
        title(['t = ', num2str(t)]);

        hold on    
        % road
        plot([-200 4000],[3.2,3.2],'k-','LineWidth',3);
        plot([-200 4000],[5,5],'k--','LineWidth',3);
        plot([-200 4000],[6.8,6.8],'k-','LineWidth',3);
        axis([xEgoA1-100 xEgoA1+100 3 7])
        car1 = scatter(xEgoA1, yEgoA1 + 4.1, msiz, 'sb',...
            'MarkerFaceColor', 'b','MarkerEdgeColor','b');
        car2 = scatter(xEgoA1+hA1, 4.1, msiz, 's',...
            'MarkerFaceColor', 'k', 'MarkerEdgeColor','k');
        
        text(xEgoA1, 5.5 + xA1(2),'ego1 wo sp','FontSize',fsiz,...
            'Color','b','HorizontalAlignment', 'center');
        text(xEgoA1+hA1, 4.3,'cau1','FontSize',fsiz,...
            'Color','k','HorizontalAlignment', 'center');
        
%         car1.MarkerFaceAlpha = malpha;
        car2.MarkerFaceAlpha = malpha;

%         subplot(212)
%         hold on
%         plot(XA(3,1:max(index-1,1)), 'r-');
%         plot(XA(3,1:max(index-1,1)), 'b--');
%         plot([0 time_horizon/con.dt], [0 0], 'k')
%         plot([0 time_horizon/con.dt], [4 4], 'k--')
%         ylim([0 5])        
%         drawnow;
    end
    % keep history of the states

    XA1(:,index) = [vEgoA1;yEgoA1;hA1;vLeadA1];

    % put your controller here
%     u_c1 = mpc_tailgate(xA1(1:3),xA1(4),con,t);
    u_c1 = mpc_supervisory(xA1,CIS, pre_XU, con, t);
    U_f1 = get_input(pre_XU,xA1,dim);
    u_c1 = u_c1(:,1);
%     u_c1 = mpc_supervisory(xA1,CIS_bnd, preXU_bnd, con);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    
    % plot input region
    subplot(212);
    plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
    hold on;
    plot(U_f1,'Color','b');
    plot(u_c1(1),u_c1(2),'og','markersize',15);
    hold off;
    title("input of ego1");
    drawnow;
    % make sure the car respects velocity and acceleration bounds
    if abs(hA1) < con.h_reaction
        deltaA1 = dual_delta(xA1, [aEgoA1;vyEgoA1], UnSafe, con, "ann");
        aLeadA1 = min(max(con.K_cau*xA1-con.K_cau(4)*con.vL_des+deltaA1, con.aL_min), con.aL_max);
    else
        aLeadA1 = min(max(-(vLeadA1-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA1 + aLeadA1 * con.dt > con.vL_max
        aLeadA1 = (con.vL_max - vLeadA1)/con.dt;
    elseif vLeadA1 + aLeadA1 * con.dt < con.vL_min
        aLeadA1 = (con.vL_min - vLeadA1)/con.dt;
    end
    % state updates
    vEgoA1 = vEgoA1 - con.f1*vEgoA1*con.dt + aEgoA1*con.dt + wx(index);
    yEgoA1 = yEgoA1 + vyEgoA1*con.dt + wy(index);
    hA1 = hA1 + (vLeadA1 - vEgoA1)*con.dt;
    vLeadA1 = vLeadA1 - con.f1*vLeadA1*con.dt + aLeadA1*con.dt + wL(index);
    xA1 = [vEgoA1; yEgoA1; hA1; vLeadA1];
    X_list = [X_list, xA1];
    U_list = [U_list,u_c1];
    T_list = [T_list, t];
%     print(fig,['./pic/frame',num2str(counter-1)],'-dpng');
end
save traj_cau.mat X_list U_list T_list
