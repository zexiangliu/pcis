clc; clear all; close all;
load CIS_collection_v2.mat


[dyn_a , dyn_c] = get_takeover_pwd();

preXU_ann = pre_xu(dyn_a,CIS_ann,0);

U = get_input(preXU_ann,[25;0;4;25]);

plot(U)

preXU_cau = pre_xu(dyn_c,CIS_cau,0);

U = get_input(preXU_cau,[25;0;4;25]);

plot(U)

%%
close all;
% U = get_input(preXU_ann,[25;0;4;25]);
% figure
% plot(U)

% U = get_input(preXU_ann,[25;0.9;0;25]);
% hold on;
% plot(U)

U = get_input(preXU_cau,[24.0001;0;-4;24]);
hold on;
plot(U)
