% plot the steering and throttle of the MPC controller over time
% plot the safe input set over the MPC inputs
clc;clear all;close all;
filename = "exp8";
load(filename);
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
expr = exp8;

X_list = expr.X_list;
U_list = expr.U_list;
T_list = expr.T_list;
US_list = expr.US_list;

step = size(U_list,2);
preXU = preXU_cau;


sizU = size(U_list,2);

U1_list1 = [];
U1_list2 = [];
ind1_list = [];
U2_list1 = [];
U2_list2 = [];
ind2_list = [];
U3_list1 = [];
U3_list2 = [];
ind3_list = [];
U4_list1 = [];
U4_list2 = [];
ind4_list = [];

for i = 1:step
   i
   x = X_list(:,i);
   u = U_list(:,i);
   t = T_list(i);
   pre_xu1 = preXU_bnd.slice([1,2,3],x(1:3));
   pre_xu2 = US_list(i);
   if ~isempty(pre_xu1.Set)
       u1 = pre_xu1.slice(2,u(2));
       if u1.Num >= 1
           [v1,v2] = find_V(u1);
           U1_list1 = [U1_list1, v1];
           U1_list2 = [U1_list2, v2];
           ind1_list = [ind1_list,i];
       end
       % steering; bnd vel
       u2 = pre_xu1.slice(1,u(1));
       if u2.Num >= 1
           [v1,v2] = find_V(u2);
           U2_list1 = [U2_list1, v1];
           U2_list2 = [U2_list2, v2];
           ind2_list = [ind2_list,i];
       end
   end
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
end
%%
save("graph"+filename);
%%
% clc;clear all;close all;
% load graphexp1.mat
% expr=exp2;
load graphexp3.mat
expr=exp3;
t_detection = expr.t_detection;
fig = figure('position',[100 100 840 300]);
wl = 3.5;
subplot(221)
hold on;
title = "throttle/bounded velocity inv set";
ylabel = "$a_{e,x}~ (m/s^2)$";
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',wl);
plot_traj(ind1_list,U1_list1,U1_list2,T_list,title,ylabel);
plot([t_detection,t_detection],[-3,3],'--c','linewidth',2)
subplot(222)
hold on;
title = "steering/bounded velocity inv set";
ylabel = "$v_{e,y}~ (m/s)$";
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',wl);
plot_traj(ind2_list,U2_list1,U2_list2,T_list,title,ylabel);
plot([t_detection,t_detection],[-1.8,1.8],'--c','linewidth',2)
subplot(223)
hold on;
title = "throttle/cautious driver inv set";
ylabel = "$a_{e,x}~ (m/s^2)$";
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',wl);
plot_traj(ind3_list,U3_list1,U3_list2,T_list,title,ylabel);
plot([t_detection,t_detection],[-3,3],'--c','linewidth',2)
subplot(224)
hold on;
title = "steering/cautious driver inv set";
ylabel = "$v_{e,y}~ (m/s)$";
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',wl);
plot_traj(ind4_list,U4_list1,U4_list2,T_list,title,ylabel)
plot([t_detection,t_detection],[-1.8,1.8],'--c','linewidth',2)

