% Compare two cautious drivers with ego car supervised or not supervised.
% better visual
clear all;close all;clc;
load CIS_bnd.mat
load CIS_bnd_XU.mat
x0 = [30 0.1 22 25]';
xC1 = x0;
xC2 = x0;
% function simulate_intention(x0)
con = constants_tri;
time_horizon = 10;
plot_stuff = 1;
% initial conditions
vEgoC1 = x0(1); yEgoC1 = x0(2); hC1 = x0(3); vLeadC1 = x0(4);
vEgoC2 = x0(1); yEgoC2 = x0(2); hC2 = x0(3); vLeadC2 = x0(4);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
XC1 = zeros(4,time_horizon/con.dt+1);
XC2 = zeros(4,time_horizon/con.dt+1);

% simulate
figure;

counter = 1;
for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoC1 = -95 + sum(XC1(1,:))*con.dt;
    xEgoC2 = -95 + sum(XC2(1,:))*con.dt;
    if plot_stuff
        %figure;
        clf; 
        title(['t = ', num2str(t)]);

%         subplot(211)
        hold on    
        % road
        plot([-200 4000],[3.2,3.2],'k-','LineWidth',3);
        plot([-200 4000],[6.4,6.4],'k--','LineWidth',3);
        plot([-200 4000],[9.8,9.8],'k-','LineWidth',3);
        axis([xEgoC1-100 xEgoC1+100 3 10])
        plot(xEgoC1, yEgoC1 + 4.8, 'sb', 'markersize', 15,'MarkerFaceColor', 'b');
        plot(xEgoC1+hC1, 4.8, 'sr', 'markersize', 15,'MarkerFaceColor', 'r');
        text(xEgoC1+hC1, 4.3,'cautious1','HorizontalAlignment', 'center');
        text(xEgoC1, 5.5 + xC1(2),'ego wo sp','HorizontalAlignment', 'center');

        % cautious scenario
        
        plot(xEgoC2, yEgoC2 + 4.8, 'db', 'markersize', 15,'MarkerFaceColor', 'g');
        plot(xEgoC2 + hC2, 4.8, 'dr', 'markersize', 15, 'MarkerFaceColor', 'k');
        text(xEgoC2 + hC2, 4 , 'cautious2', 'HorizontalAlignment', 'center');
        text(xEgoC2, 6 + xC2(2),'ego wt sp','HorizontalAlignment', 'center');

        
%         subplot(212)
%         hold on
%         plot(XA(3,1:max(index-1,1)), 'r-');
%         plot(XC(3,1:max(index-1,1)), 'b--');
%         plot([0 time_horizon/con.dt], [0 0], 'k')
%         plot([0 time_horizon/con.dt], [4 4], 'k--')
%         ylim([0 5])        
        drawnow;
    end
    M(counter) = getframe;
    counter = counter + 1;
    % keep history of the states

    XC1(:,index) = [vEgoC1;yEgoC1;hC1;vLeadC1];
    XC2(:,index) = [vEgoC2;yEgoC2;hC2;vLeadC2];

    % put your controller here
    u_c1 = mpc_simple(xC1(1:3),xC1(4),con);
%     u_c1 = mpc_supervisory(xC1,CIS_bnd, preXU_bnd, con);
    aEgoC1 = u_c1(1,1); 
    vyEgoC1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoC1 =  max(min(aEgoC1, con.umax_ACC), con.umin_ACC);
    vyEgoC1 =  max(min(vyEgoC1, con.umax_LK), con.umin_LK);
    
    % put your controller here
%     u_c2 = mpc_simple(xC2(1:3),xC2(4),con);
    u_c2 = mpc_supervisory(xC2,CIS_bnd, preXU_bnd, con);
    aEgoC2 = u_c2(1,1); 
    vyEgoC2 = u_c2(2,1);
    % make sure inputs satisfy limits
    aEgoC2 =  max(min(aEgoC2, con.umax_ACC), con.umin_ACC);
    vyEgoC2 =  max(min(vyEgoC2, con.umax_LK), con.umin_LK);
    
    
%     % ego car lateral input (overtaking manouvre)
%     if (hA >= 0 && hA/vEgoA < con.tau_des) || (hA < 0 && -hA/vLeadA < con.tau_min)
%         vyEgoA = min(con.umax_LK, con.y_max-con.y_lane/2 - yEgoA);
%     else
%         vyEgoA = max(con.umin_LK, -yEgoA);
%     end
%     
%     if (hC >= 0 && hC/vEgoC < con.tau_des) || (hC < 0 && -hC/vLeadC < con.tau_min)
%         vyEgoC = min(con.umax_LK, con.y_max - con.y_lane/2 - yEgoC);
%     else
%         vyEgoC = max(con.umin_LK, -yEgoC);
%     end
    
    % make sure annoying car respects velocity and acceleration bounds
    if abs(hC1) < con.h_reaction
        aLeadC1 = min(max(con.K_cau*xC1, con.aL_min), con.aL_max);
    else
        aLeadC1 = min(max(-(vLeadC1-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadC1 + aLeadC1 * con.dt > con.vL_max
        aLeadC1 = (con.vL_max - vLeadC1)/con.dt;
    elseif vLeadC1 + aLeadC1 * con.dt < con.vL_min
        aLeadC1 = (con.vL_min - vLeadC1)/con.dt;
    end
    % make sure cautious car respects velocity and acceleration bounds
    if abs(hC2) < con.h_reaction
        aLeadC2 = min(max(con.K_cau*xC2, con.aL_min), con.aL_max);
    else
        aLeadC2 = min(max(-(vLeadC2-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadC2 + aLeadC2 * con.dt > con.vL_max
        aLeadC2 = (con.vL_max - vLeadC2)/con.dt;
    elseif vLeadC2 + aLeadC2 * con.dt < con.vL_min
        aLeadC2 = (con.vL_min - vLeadC2)/con.dt;
    end
    
    % state updates
    vEgoC1 = vEgoC1 - con.f1*vEgoC1*con.dt + aEgoC1*con.dt + wx(index);
    yEgoC1 = yEgoC1 + vyEgoC1*con.dt + wy(index);
    hC1 = hC1 + (vLeadC1 - vEgoC1)*con.dt;
    vLeadC1 = vLeadC1 - con.f1*vLeadC1*con.dt + aLeadC1*con.dt + wL(index);
    xC1 = [vEgoC1; yEgoC1; hC1; vLeadC1];
    
    vEgoC2 = vEgoC2 - con.f1*vEgoC2*con.dt + aEgoC2*con.dt + wx(index);
    yEgoC2 = yEgoC2 + vyEgoC2*con.dt + wy(index);
    hC2 = hC2 + (vLeadC2 - vEgoC2)*con.dt;
    vLeadC2 = vLeadC2 - con.f1*vLeadC2*con.dt + aLeadC2*con.dt + wL(index);
    xC2 = [vEgoC2; yEgoC2; hC2; vLeadC2];

        
end

