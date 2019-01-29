clc;clear all;close all;
% %%
load CIS_bnd.mat
load CIS_inv_R7.mat
load CIS_bnd_XU.mat
% % 
% [dyn_a , dyn_c] = get_takeover_pwd();
% rho = 0;
mptopt('lpsolver', 'CDD', 'qpsolver', 'GUROBI');

% visual(CIS_ann);
% visual(CIS_cau);

%%
% str_x = "(";
% for i = 1:3
%     str_x = str_x+num2str(x(i),'%.2f')+",";
% end
% str_x(end) = str_x+num2str(x(4),'%.2f')+")";
fig1 = figure(3);
% fig2 = figure(4);
% fig3 = figure(5);
% [x,u] = get_from_traj(idx);
u = [0;0];
x = [25; -0.297; 16.52; 20];
containsPolyUnion(CIS_cau,x)
containsPolyUnion(CIS_ann,x)
containsPolyUnion(CIS_bnd,x(1:3))

%%
% str_x = num2str(idx);
subplot(131);
plotU(x,preXU_ann,fig1)
% plot(u(1),u(2),'x','Markersize',15);
%     print("s_"+str_x+"/"+"ann_"+str_x,'-dpng');
xlabel("$a_{e,x}$", 'interpreter','latex')
ylabel("$v_{e,y}$", 'interpreter','latex')
set(gca,"fontsize",12)
title("agressive driver", 'interpreter','latex')

subplot(132);
% mkdir("s_"+str_x)
plotU(x,preXU_cau,fig1);
% hold on;
xlabel("$a_{e,x}$", 'interpreter','latex')
ylabel("$v_{e,y}$", 'interpreter','latex')
set(gca,"fontsize",12)
title("cautious driver", 'interpreter','latex');
% plot(u(1),u(2),'x','Markersize',15);
%     print("s_"+str_x+"/"+"cau_"+str_x,'-dpng');
subplot(133);
plotU(x(1:3),preXU_bnd,fig1)
xlabel("$a_{e,x}$", 'interpreter','latex')
ylabel("$v_{e,y}$", 'interpreter','latex')
set(gca,"fontsize",12)
title("bounded velocity", 'interpreter','latex');
% plot(u(1),u(2),'x','Markersize',15);
drawnow;
%     print("s_"+str_x+"/"+"ann_"+str_x,'-dpng');

% safeU = get_safe_inputs('ac', x, CIS_ac, dyn_a, dyn_c);
%%
% figure;
% plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
% hold on;
% plot(safeU, 'color', 'g');

