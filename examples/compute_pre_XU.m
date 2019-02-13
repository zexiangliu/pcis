% load CIS_ann_R7.mat
% 
% [dyn_a , dyn_c] = get_takeover_pwd_7regions();
% 
% rho = 0;
% CIS_ann_XU = dyn_a.pre_xu(CIS_ann, rho)
% 
% save CIS_ann_XU_R7.mat CIS_ann_XU

clc;clear all;close all;
load CIS_cau_R7_v2.mat
mptopt('lpsolver', 'GUROBI', 'qpsolver', 'GUROBI');

[dyn_a , dyn_c] = get_takeover_pwd_7regions();

rho = 0;
preXU_cau = dyn_c.pre_xu(CIS_cau, rho)

save CIS_cau_XU_R7.mat preXU_cau
