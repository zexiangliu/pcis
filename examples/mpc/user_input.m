% plot the steering and throttle of the MPC controller over time
% plot the safe input set over the MPC inputs
clc;clear all;close all;
filename = "traj4";
load(filename);
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat

%%
step = size(U_list,2);
preXU = preXU_cau;
con = constants_tri();

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

t_detection = 0;
Iv = "bnd";

t_list = 0:0.1:T_list(end);
step = length(t_list)-1;

x1 = @(t) interp1(T_list,X_list(1,:),t);
x2 = @(t) interp1(T_list,X_list(2,:),t);
x3 = @(t) interp1(T_list,X_list(3,:),t);
x4 = @(t) interp1(T_list,X_list(4,:),t);
X = @(t) [x1(t);x2(t);x3(t);x4(t)];
u1 = @(t) interp1(T_list,U_list(1,:),t);
u2 = @(t) interp1(T_list,U_list(2,:),t);
U = @(t) [u1(t);u2(t)];

for i = 1:step
   i
   t = t_list(i);
   x = X(t);
   u = U(t);
   
%    containsPolyUnion(CIS_bnd,x(1:3))
   
   if Iv == "bnd"
       aLeadA1 = (x4(t+con.dt)-x(4)+con.f1*x(4)*con.dt)/con.dt;
       Iv = intention_estimation(x, aLeadA1, con);
       t_detection = t;
   end
   pre_xu1 = preXU_bnd.slice([1,2,3],x(1:3));
   pre_xu2 = preXU.slice([1,2,3,4],x);
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
   elseif ~isempty(pre_xu1.Set)
       u3 = pre_xu1.slice(2,u(2));
       if u3.Num >= 1
           [v1,v2] = find_V(u3);
           U3_list1 = [U3_list1, v1];
           U3_list2 = [U3_list2, v2];
           ind3_list = [ind3_list,i];
       end
       % throttle; cau
       u4 = pre_xu1.slice(1,u(1));
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
% filename = "traj1";
% load("graph"+filename);
% load("graphtraj2.mat");
fig = figure('position',[100 100 840 300]);
wl = 3.5;
subplot(221)
hold on;
title = "throttle/bounded velocity inv set";
ylabel = "$a_{e,x}$";
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',wl);
plot_traj(ind1_list,U1_list1,U1_list2,t_list,title,ylabel);
plot([t_detection,t_detection],[-3,3],'--g','linewidth',2)
subplot(222)
hold on;
title = "steering/bounded velocity inv set";
ylabel = "$v_{e,y}$";
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',wl);
plot_traj(ind2_list,U2_list1,U2_list2,t_list,title,ylabel);
plot([t_detection,t_detection],[-1.8,1.8],'--g','linewidth',2)
subplot(223)
hold on;
title = "throttle/cautious driver inv set";
ylabel = "$a_{e,x}$";
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',wl);
plot_traj(ind3_list,U3_list1,U3_list2,t_list,title,ylabel);
plot([t_detection,t_detection],[-3,3],'--g','linewidth',2)
subplot(224)
hold on;
title = "steering/cautious driver inv set";
ylabel = "$v_{e,y}$";
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',wl);
plot_traj(ind4_list,U4_list1,U4_list2,t_list,title,ylabel)
plot([t_detection,t_detection],[-1.8,1.8],'--g','linewidth',2)

