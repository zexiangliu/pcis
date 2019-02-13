% plot the steering and throttle of the MPC controller over time
% plot the safe input set over the MPC inputs
clc;clear all;close all;
filename = "traj1";
load(filename);

[dyn_a , dyn_c] = get_takeover_pwd_7regions();

step = size(U_list,2);
%%
con = constants_tri();

UnSafe = Polyhedron('UB',[inf inf con.h_min, inf],'LB',...
    -[inf inf con.h_min, inf]);

sizU = size(U_list,2);
t_detection = 0;
X_NEW_list = zeros(size(X_list));
X_NEW_list(:,1) = X_list(:,1);
for i = 1:step-1
   x = X_NEW_list(:,i);
   u = U_list(:,i);
   dt = (T_list(i+1)-T_list(i));
   con.dt = dt;
   [X_NEW_list(:,i+1),Iv] = update_dyn(x,u,[0;0;0],UnSafe,con,"cau");
   Iv
   aLeadA1 = (X_list(4,i+1)-X_list(4,i)+con.f1*X_list(4,i)*dt)/dt;
   Iv = intention_estimation(X_list(:,i), aLeadA1, con)
end

%%
figure;
plot(X_list(1,:));hold on;
hold on;
plot(X_NEW_list(1,:));

figure;
plot(U_list(1,:))