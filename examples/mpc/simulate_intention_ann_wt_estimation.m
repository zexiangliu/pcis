% Compare two aggressive drivers with ego car supervised or not supervised.
% better visual
% solve dual delta
clear all;close all;clc;
% load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
load CIS_bnd.mat
mkdir pic
%CIS = CIS_ann;
%pre_XU = preXU_ann;
Iv = 'bnd';
Iv_CIS = CIS_bnd.copy();
Iv_preXU = preXU_bnd.copy();
dim = Iv_CIS.Set(1).Dim;

x0 = [30 0.1 22 25]';
if ~containsPolyUnion(Iv_CIS,x0(1:3))
    error("bad initial point!")
end

xA1 = x0;
xA2 = x0;
% function simulate_intention(x0)
con = constants_tri;
time_horizon = 20;
plot_stuff = 1;


UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

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
fig = figure('position',[100 100 1200 640]);
msiz = 200; % marker size
malpha = 0.5; % marker alpha
fsiz = 12;
counter = 1;


for t = 0:con.dt:time_horizon
    % visualize
    index = round(t/con.dt+1);
    xEgoA1 = -95 + sum(XA1(1,:))*con.dt;
    xEgoA2 = -95 + sum(XA2(1,:))*con.dt;
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
        
        text(xEgoA1, 5.2+ xA1(2),'ego1 wo sp','FontSize',fsiz,...
            'Color','b','HorizontalAlignment', 'center');
        text(xEgoA1+hA1, 3.4,'agg1','FontSize',fsiz,...
            'Color','k','HorizontalAlignment', 'center');
        
        % annoying scenario
        
        car3 = scatter(xEgoA2, yEgoA2 + 4.1, msiz, 'd',...
            'MarkerFaceColor', 'g','MarkerEdgeColor','g');
        car4 = scatter(xEgoA2 + hA2, 4.1, msiz, 'd',...
            'MarkerFaceColor', 'r','MarkerEdgeColor','r');
        
        text(xEgoA2, 4.6 + xA2(2),'ego2 wt sp','FontSize',fsiz,...
            'Color','g','HorizontalAlignment', 'center');
        text(xEgoA2 + hA2, 3.8 , 'agg2','FontSize',fsiz, ...
            'Color','r','HorizontalAlignment', 'center');

%         car1.MarkerFaceAlpha = malpha;
        car2.MarkerFaceAlpha = malpha;
        car3.MarkerFaceAlpha = malpha;
        car4.MarkerFaceAlpha = malpha;

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
    XA2(:,index) = [vEgoA2;yEgoA2;hA2;vLeadA2];

    % put your controller here
    u_c1 = mpc_tailgate(xA1(1:3), xA1(4), con, t);
    U_f1 = get_input(Iv_preXU,xA1,dim);
    u_c1 = u_c1(:,1);
%     u_c1 = mpc_supervisory(xA1,CIS_bnd, preXU_bnd, con);
    aEgoA1 = u_c1(1,1); 
    vyEgoA1 = u_c1(2,1);
    % make sure inputs satisfy limits
    aEgoA1 =  max(min(aEgoA1, con.umax_ACC), con.umin_ACC);
    vyEgoA1 =  max(min(vyEgoA1, con.umax_LK), con.umin_LK);
    
    % put your controller here
%     u_c2 = mpc_simple(xA2(1:3),xA2(4),con);
%     ud_c2 = u_c2;
%     U_f2 = get_input(preXU_bnd,xA1,3);
    [u_c2, ud_c2, ~, U_f2] = mpc_supervisory(xA2,Iv_CIS, Iv_preXU, con, t);
    
    % plot input region
    subplot(312);
    plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
    hold on;
    plot(U_f1,'Color','b','edgealpha',0);
    plot(u_c1(1),u_c1(2),'og','markersize',15, 'linewidth', 2);
    hold off;
    title("input of ego1");
    subplot(313);
    plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
    hold on;
    plot(U_f2,'Color','b','edgealpha',0);
    plot(ud_c2(1),ud_c2(2),'og','markersize',15, 'linewidth', 2);
    plot(u_c2(1),u_c2(2),'.g','markersize',30);
    hold off;
    title("input of ego2")
    drawnow;

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
        deltaA1 = dual_delta(xA1, [aEgoA1;vyEgoA1], UnSafe, con, "ann");
        aLeadA1 = min(max(con.K_ann*xA1+deltaA1, con.aL_min), con.aL_max);
        if strcmp(Iv,'bnd')
            Iv = intention_estimation(xA1, aLeadA1, con);
            if strcmp(Iv, 'ann')
                Iv_CIS = CIS_ann.copy();
                Iv_preXU = preXU_ann.copy();
                dim = Iv_CIS.Set(1).Dim;
            elseif strcmp(Iv, 'cau')
                Iv_CIS = CIS_cau.copy();
                Iv_preXU = preXU_cau.copy();
                dim = Iv_CIS.Set(1).Dim;
            end
        end
        
    else
        aLeadA1 = min(max(-(vLeadA1-con.vL_des)/con.dt, con.aL_min), con.aL_max);
    end
    if vLeadA1 + aLeadA1 * con.dt > con.vL_max
        aLeadA1 = (con.vL_max - vLeadA1)/con.dt;
    elseif vLeadA1 + aLeadA1 * con.dt < con.vL_min
        aLeadA1 = (con.vL_min - vLeadA1)/con.dt;
    end
    % make sure annoying car respects velocity and acceleration bounds
    if abs(hA2) < con.h_reaction
        deltaA2 = dual_delta(xA2, [aEgoA2;vyEgoA2], UnSafe, con, "ann");
        aLeadA2 = min(max(con.K_ann*xA2+deltaA2, con.aL_min), con.aL_max);
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
    vLeadA1 = vLeadA1 - con.f1*vLeadA1*con.dt + aLeadA1*con.dt + wL(index);
    xA1 = [vEgoA1; yEgoA1; hA1; vLeadA1];
    
    vEgoA2 = vEgoA2 - con.f1*vEgoA2*con.dt + aEgoA2*con.dt + wx(index);
    yEgoA2 = yEgoA2 + vyEgoA2*con.dt + wy(index);
    hA2 = hA2 + (vLeadA2 - vEgoA2)*con.dt;
    vLeadA2 = vLeadA2 - con.f1*vLeadA2*con.dt + aLeadA2*con.dt + wL(index);
    xA2 = [vEgoA2; yEgoA2; hA2; vLeadA2];
    M(counter) = getframe(fig);
    counter = counter + 1;
    print(fig,['./pic/frame',num2str(counter-1)],'-dpng');
end

