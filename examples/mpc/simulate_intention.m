clear all;close all;clc;
load CIS_bnd.mat
load CIS_bnd_XU.mat
x0 = [25 0 15 25]';
xA = x0;
xC = x0;
% function simulate_intention(x0)
con = constants_tri;
time_horizon = 15;
plot_stuff = 1;
% initial conditions
vEgoA = x0(1); yEgoA = x0(2); hA = x0(3); vLeadA = x0(4);
vEgoC = x0(1); yEgoC = x0(2); hC = x0(3); vLeadC = x0(4);

% disturbance
wx = zeros(time_horizon/con.dt+1,1);%2*con.dmax_ACC*rand(time_horizon/con.dt+1,1)-con.dmax_ACC;
wy = zeros(time_horizon/con.dt+1,1);%2*con.dmax_LK*rand(time_horizon/con.dt+1,1)-con.dmax_LK;
wL = zeros(time_horizon/con.dt+1,1);%2*con.dLmax*rand(time_horizon/con.dt+1,1)-con.dLmax;
% history
XA = zeros(4,time_horizon/con.dt+1);
XC = zeros(4,time_horizon/con.dt+1);

% simulate
figure;

counter = 1;
for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoA = -95 + sum(XA(1,:))*con.dt;
    xEgoC = -95 + sum(XC(1,:))*con.dt;
    if plot_stuff
        %figure;
        clf; 
        title(['t = ', num2str(t)]);

%         subplot(211)
        hold on    
        % road
        plot([-200 4000],[-3.2,-3.2],'k-','LineWidth',3);
        plot([-200 4000],[-6.4,-6.4],'k--','LineWidth',3);
        plot([-200 4000],[-9.8,-9.8],'k-','LineWidth',3);
        plot([-200 4000],[3.2,3.2],'k-','LineWidth',3);
        plot([-200 4000],[6.4,6.4],'k--','LineWidth',3);
        plot([-200 4000],[9.8,9.8],'k-','LineWidth',3);
        axis([xEgoA-100 xEgoA+100 -10 10])
        plot(xEgoA, yEgoA + 4.8, 'sb', 'markersize', 5,'MarkerFaceColor', 'b');
        plot(xEgoA+hA, 4.8, 'sr', 'markersize', 5,'MarkerFaceColor', 'r');
        text(xEgoA+hA, 4.3,'annoying','HorizontalAlignment', 'center');

        % cautious scenario
        
        plot(xEgoC, yEgoC - 8.1, 'db', 'markersize', 5,'MarkerFaceColor', 'b');
        plot(xEgoC + hC, -8.1, 'dr', 'markersize', 5, 'MarkerFaceColor', 'r');
        text(xEgoC + hC, -8.6 , 'cautious', 'HorizontalAlignment', 'center');

        
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

    XA(:,index) = [vEgoA;yEgoA;hA;vLeadA];
    XC(:,index) = [vEgoC;yEgoC;hC;vLeadC];

    % put your controller here
    u_a = mpc_simple(xA(1:3),xA(4),con);
%     u_a = mpc_supervisory(xA,CIS_bnd, preXU_bnd, con);
    aEgoA = u_a(1,1); 
    vyEgoA = u_a(2,1);
    % make sure inputs satisfy limits
    aEgoA =  max(min(aEgoA, con.umax_ACC), con.umin_ACC);
    vyEgoA =  max(min(vyEgoA, con.umax_LK), con.umin_LK);
    
    % put your controller here
    u_c = mpc_simple(xC(1:3),xC(4),con);
%     u_c = mpc_supervisory(xC,CIS_bnd, preXU_bnd, con);
    aEgoC = u_c(1,1); 
    vyEgoC = u_c(2,1);
    % make sure inputs satisfy limits
    aEgoC =  max(min(aEgoC, con.umax_ACC), con.umin_ACC);
    vyEgoC =  max(min(vyEgoC, con.umax_LK), con.umin_LK);
    
    
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
    if abs(hA) < con.h_reaction
        aLeadA = min(max(con.K_ann*xA, con.aL_min), con.aL_max)
    else
        aLeadA = min(max(-(vLeadA-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA + aLeadA * con.dt > con.vL_max
        aLeadA = (con.vL_max - vLeadA)/con.dt;
    elseif vLeadA + aLeadA * con.dt < con.vL_min
        aLeadA = (con.vL_min - vLeadA)/con.dt;
    end
    % make sure cautious car respects velocity and acceleration bounds
    if abs(hC) < con.h_reaction
        aLeadC = min(max(con.K_cau*xC, con.aL_min), con.aL_max)
    else
        aLeadC = min(max(-(vLeadC-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadC + aLeadC * con.dt > con.vL_max
        aLeadC = (con.vL_max - vLeadC)/con.dt;
    elseif vLeadC + aLeadC * con.dt < con.vL_min
        aLeadC = (con.vL_min - vLeadC)/con.dt;
    end
    
    % state updates
    vEgoA = vEgoA - con.f1*vEgoA*con.dt + aEgoA*con.dt + wx(index);
    yEgoA = yEgoA + vyEgoA*con.dt + wy(index);
    hA = hA + (vLeadA - vEgoA)*con.dt;
    vLeadA = vLeadA - con.f2*vLeadA*con.dt + aLeadA*con.dt + wL(index);
    xA = [vEgoA; yEgoA; hA; vLeadA];
    
    vEgoC = vEgoC - con.f1*vEgoC*con.dt + aEgoC*con.dt + wx(index);
    yEgoC = yEgoC + vyEgoC*con.dt + wy(index);
    hC = hC + (vLeadC - vEgoC)*con.dt;
    vLeadC = vLeadC - con.f1*vLeadC*con.dt + aLeadC*con.dt + wL(index);
    xC = [vEgoC; yEgoC; hC; vLeadC];

        
end

