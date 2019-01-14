clear all;close all;clc;
load CIS_bnd.mat
load CIS_bnd_XU.mat
x0 = [25 0 15 25]';
xA1 = x0;
xA2 = x0;
% function simulate_intention(x0)
con = constants_tri;
time_horizon = 10;
plot_stuff = 1;
% initial conditions
vEgoA1 = x0(1); yEgoA1 = x0(2); hA1 = x0(3); vLeadA1 = x0(4);
vEgoA2 = x0(1); yEgoA2 = x0(2); hA2 = x0(3); vLeadA2 = x0(4);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
XA1 = zeros(4,time_horizon/con.dt+1);
XA2 = zeros(4,time_horizon/con.dt+1);

% simulate
figure;

counter = 1;
for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoA1 = -95 + sum(XA1(1,:))*con.dt;
    xEgoA2 = -95 + sum(XA2(1,:))*con.dt;
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
        axis([xEgoA1-100 xEgoA1+100 3 10])
        plot(xEgoA1, yEgoA1 + 4.8, 'sb', 'markersize', 15,'MarkerFaceColor', 'b');
        plot(xEgoA1+hA1, 4.8, 'sr', 'markersize', 15,'MarkerFaceColor', 'r');
        text(xEgoA1+hA1, 4.3,'annoying1','HorizontalAlignment', 'center');
        text(xEgoA1, 5.5 + xA1(2),'ego wo sp','HorizontalAlignment', 'center');

        % cautious scenario
        
        plot(xEgoA2, yEgoA2 + 4.8, 'db', 'markersize', 15,'MarkerFaceColor', 'g');
        plot(xEgoA2 + hA2, 4.8, 'dr', 'markersize', 15, 'MarkerFaceColor', 'k');
        text(xEgoA2 + hA2, 4 , 'annoying2', 'HorizontalAlignment', 'center');
        text(xEgoA2, 6 + xA2(2),'ego wt sp','HorizontalAlignment', 'center');

        
%         subplot(212)
%         hold on
%         plot(XA(3,1:max(index-1,1)), 'r-');
%         plot(XA(3,1:max(index-1,1)), 'b--');
%         plot([0 time_horizon/con.dt], [0 0], 'k')
%         plot([0 time_horizon/con.dt], [4 4], 'k--')
%         ylim([0 5])        
        drawnow;
    end
    M(counter) = getframe;
    counter = counter + 1;
    % keep history of the states

    XA1(:,index) = [vEgoA1;yEgoA1;hA1;vLeadA1];
    XA2(:,index) = [vEgoA2;yEgoA2;hA2;vLeadA2];

    % put your controller here
    u_c1 = mpc_simple(xA1(1:3),xA1(4),con);
%     u_c1 = mpc_supervisory(xA1,CIS_bnd, preXU_bnd, con);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    
    % put your controller here
%     u_c2 = mpc_simple(xA2(1:3),xA2(4),con);
    u_c2 = mpc_supervisory(xA2,CIS_bnd, preXU_bnd, con);
    aEgoA2 = u_c2(1,1); 
    vyEgoA2 = u_c2(2,1);
    % make sure inputs satisfy limits
    aEgoA2 =  max(min(aEgoA2, con.umax_ACC), con.umin_ACC);
    vyEgoA2 =  max(min(vyEgoA2, con.umax_LK), con.umin_LK);
    
    
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
    if abs(hA1) < con.h_reaction
        aLeadA1 = min(max(-con.K_ann*xA1, con.aL_min), con.aL_max);
    else
        aLeadA1 = min(max(-(vLeadA1-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA1 + aLeadA1 * con.dt > con.vL_max
        aLeadA1 = (con.vL_max - vLeadA1)/con.dt;
    elseif vLeadA1 + aLeadA1 * con.dt < con.vL_min
        aLeadA1 = (con.vL_min - vLeadA1)/con.dt;
    end
    % make sure cautious car respects velocity and acceleration bounds
    if abs(hA2) < con.h_reaction
        aLeadA2 = min(max(-con.K_ann*xA2, con.aL_min), con.aL_max);
    else
        aLeadA2 = min(max(-(vLeadA2-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA2 + aLeadA2 * con.dt > con.vL_max
        aLeadA2 = (con.vL_max - vLeadA2)/con.dt;
    elseif vLeadA2 + aLeadA2 * con.dt < con.vL_min
        aLeadA2 = (con.vL_min - vLeadA2)/con.dt;
    end
    
    % state updates
    vEgoA1 = vEgoA1 - con.f1*vEgoA1*con.dt + aEgoA1*con.dt + wx(index);
    yEgoA1 = yEgoA1 + vyEgoA1*con.dt + wy(index);
    hA1 = hA1 + (vLeadA1 - vEgoA1)*con.dt;
    vLeadA1 = vLeadA1 - con.f2*vLeadA1*con.dt + aLeadA1*con.dt + wL(index);
    xA1 = [vEgoA1; yEgoA1; hA1; vLeadA1];
    
    vEgoA2 = vEgoA2 - con.f1*vEgoA2*con.dt + aEgoA2*con.dt + wx(index);
    yEgoA2 = yEgoA2 + vyEgoA2*con.dt + wy(index);
    hA2 = hA2 + (vLeadA2 - vEgoA2)*con.dt;
    vLeadA2 = vLeadA2 - con.f1*vLeadA2*con.dt + aLeadA2*con.dt + wL(index);
    xA2 = [vEgoA2; yEgoA2; hA2; vLeadA2];

        
end

