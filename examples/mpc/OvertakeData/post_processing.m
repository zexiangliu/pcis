clc;clear all;close all;
load AIState_ann_2.mat
load driverState_ann_2.mat

T_list = [state.time]';

v_ex = state.Data(:,3); 
y_e = 3.2 - state.Data(:,2);
idx = y_e == 2.7 | y_e == -0.9;
h = intentionState.Data(:,1) - state.Data(:,1);
vLead = intentionState.Data(:,3);

u1 = state.Data(:,6);
u2 = -state.Data(:,4);
% sum(idx)
u2(idx) = 0;
X_list = [v_ex,y_e,h, vLead]';
U_list = [u1,u2]';

figure;
plot(T_list,v_ex);
hold on; plot(T_list,u1);

figure;
plot(T_list,y_e);
hold on ;plot(T_list,u2);

save traj1.mat