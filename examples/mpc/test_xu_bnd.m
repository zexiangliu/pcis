clc;clear all;close all;
% %%
load CIS_bnd.mat
load CIS_inv_R7.mat
load CIS_bnd_XU.mat
% % 
% [dyn_a , dyn_c] = get_takeover_pwd();
% rho = 0;
mptopt('lpsolver', 'CDD', 'qpsolver', 'GUROBI');



%%
% str_x = "(";
% for i = 1:3
%     str_x = str_x+num2str(x(i),'%.2f')+",";
% end
% str_x(end) = str_x+num2str(x(4),'%.2f')+")";
fig1 = figure(1);
fig2 = figure(2);
fig3 = figure(3);
for  idx = 500:900
    [x,u] = get_from_traj(idx);
 
    str_x = num2str(idx);

    mkdir("s_"+str_x)
    plotU(x,CIS_cau_XU,fig1);
    hold on;
    plot(u(1),u(2),'x','Markersize',15);
%     print("s_"+str_x+"/"+"cau_"+str_x,'-dpng');

    plotU(x,CIS_ann_XU,fig2)
    plot(u(1),u(2),'x','Markersize',15);
%     print("s_"+str_x+"/"+"ann_"+str_x,'-dpng');

    plotU(x(1:3),preXU_bnd,fig3)
    plot(u(1),u(2),'x','Markersize',15);
    drawnow;
%     print("s_"+str_x+"/"+"ann_"+str_x,'-dpng');
end

% safeU = get_safe_inputs('ac', x, CIS_ac, dyn_a, dyn_c);
%%
% figure;
% plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
% hold on;
% plot(safeU, 'color', 'g');

