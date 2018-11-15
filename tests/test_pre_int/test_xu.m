clc;clear all;close all;
% %%
load CIS_ac.mat
load CIS_collection_XU_v2_6iter.mat
load CIS_ac_XU.mat
load sample_trajectory.mat
% % 
% [dyn_a , dyn_c] = get_takeover_pwd();
% rho = 0;
mptopt('lpsolver', 'CDD', 'qpsolver', 'LCP');


%%
% 
% X1 = merge_cvx(V_X1);
% [~,preXU_unknown_X1] = pre_int_cvx(dyn_c, dyn_a, V_all, rho, [], [], false, true);
% 
% 
% 
% X2 = merge_cvx(V_X2);
% [~,preXU_unknown_X2] = pre_int_cvx(dyn_c, dyn_a, X2, rho, [], [], false, true);

%%
% 
% tmp_preXU_unknown = merge_cvx(preXU_unknown)
% 
% preXU_unknown = PolyUnion([preXU_unknown_X1.Set,preXU_unknown_X2.Set])

%%
% plotU(x,preXU_unknown)


%%

clc; %close all;
% x = [25; -0.0; -10.0; 25];

% 782        1307        1451
idx = 12;
[x,u] = get_from_traj(idx);

% x =[24.1115192855201;-0.0414780139422499;40.01465372398570;23.2950177751220];
containsPolyUnion(CIS_cau,x)
containsPolyUnion(CIS_ann,x)
containsPolyUnion(CIS_ac,x)

%%
% str_x = "(";
% for i = 1:3
%     str_x = str_x+num2str(x(i),'%.2f')+",";
% end
% str_x(end) = str_x+num2str(x(4),'%.2f')+")";
str_x = num2str(idx);

mkdir("s_"+str_x)
plotU(x,preXU_cau);
hold on;
plot(u(1),u(2),'x','Markersize',15);
print("s_"+str_x+"/"+"cau_"+str_x,'-dpng');

plotU(x,preXU_ann)
plot(u(1),u(2),'x','Markersize',15);
print("s_"+str_x+"/"+"ann_"+str_x,'-dpng');

try
    plotU(x,preXU_ac)
    plot(u(1),u(2),'x','Markersize',15);
    print("s_"+str_x+"/"+"ac_"+str_x,'-dpng');
catch
    
    plot(u(1),u(2),'x','Markersize',15);
    print("s_"+str_x+"/"+"ac_"+str_x,'-dpng');
end

% safeU = get_safe_inputs('ac', x, CIS_ac, dyn_a, dyn_c);
%%
% figure;
% plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
% hold on;
% plot(safeU, 'color', 'g');

