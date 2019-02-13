% plot the steering and throttle of the MPC controller over time
% plot the safe input set over the MPC inputs
clc;clear all;close all;
filename = "exp3";
load(filename);
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
expr = exp3;

X_list = expr.X_list;
U_list = expr.U_list;
T_list = expr.T_list;
US_list = expr.US_list;
UMPC_list = expr.UMPC_list;
step = size(U_list,2);
preXU = preXU_cau;


sizU = size(U_list,2);

U3_list1 = [];
U3_list2 = [];
ind3_list = [];
U4_list1 = [];
U4_list2 = [];
ind4_list = [];
override_list = [];

for i = 1:step
   i
   x = X_list(:,i);
   u = U_list(:,i);
   umpc = UMPC_list(:,i);
   t = T_list(i);
   pre_xu2 = US_list(i);
   % throttle; cau
   if ~isempty(pre_xu2.Set)
       u3 = pre_xu2.slice(2,u(2));
       if u3.Num >= 1
           [v1,v2] = find_V(u3);
           U3_list1 = [U3_list1, v1];
           U3_list2 = [U3_list2, v2];
           ind3_list = [ind3_list,i];
       end
       % throttle; cau
       u4 = pre_xu2.slice(1,u(1));
       if u4.Num >= 1
           [v1,v2] = find_V(u4);
           U4_list1 = [U4_list1, v1];
           U4_list2 = [U4_list2, v2];
           ind4_list = [ind4_list,i];
       end
   end
   if ~all(u==umpc)
       override_list = [override_list,i];
   end
end
%%
save("graph"+filename);
%%
load graphexp3.mat
t_detection = expr.t_detection;
fig = figure('position',[100 100 840 150]);
wl = 3.5;
subplot(121)
hold on;
title = "throttle/cautious driver inv set";
ylabel = "$a_{e,x}$";
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',wl);
plot_overriding(override_list, U_list(1,:),T_list)
plot_traj(ind3_list,U3_list1,U3_list2,T_list,title,ylabel);
plot([t_detection,t_detection],[-3,3],'--g','linewidth',2)
subplot(122)
hold on;
title = "steering/cautious driver inv set";
ylabel = "$v_{e,y}$";
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',wl);
plot_overriding(override_list, U_list(2,:),T_list)
plot_traj(ind4_list,U4_list1,U4_list2,T_list,title,ylabel)
plot([t_detection,t_detection],[-1.8,1.8],'--g','linewidth',2)

