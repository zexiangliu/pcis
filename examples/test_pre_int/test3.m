X1 = load('tmp_X1.mat');
X3 = load('tmp_X3.mat');


% [~, V_X1] = pre_int(dyn_c, dyn_a, X1.V_all, rho, regions, dyns_id, false,true);
% [~, V_X3] = pre_int(dyn_c, dyn_a, X3.V_all, rho, regions, dyns_id, false,true);
% 
% %%
% 
% CIS_unknown = PolyUnion([X1.V_all.Set X3.V_all.Set]);
% 
% preXU_unknown = PolyUnion([V_X1.Set V_X3.Set]);

%%
V_X1 = X1.V_all;
V_X3 = X3.V_all;

CIS_ac = PolyUnion([V_X1.Set,V_X3.Set]);

%%
[~, preXU_X1] = pre_int(dyn_c, dyn_a, V_X1, 0, [], [], false,true);
[~, preXU_X3] = pre_int(dyn_c, dyn_a, V_X3, 0, [], [], false,true);