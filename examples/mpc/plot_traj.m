% plot the steering and throttle of the MPC controller over time
% plot the safe input set over the MPC inputs

load traj1.mat
load CIS_bnd.mat
load CIS_bnd_XU.mat
load CIS_inv_R7.mat
step = size(T_list,2)-1;
preXU = preXU_ann;

fig = figure('position',[100 100 640 500]);
sizU = size(U_list,2);
subplot(221)
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',3);
subplot(223)
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',3);
subplot(222)
plot(T_list(1:sizU),U_list(1,:),'r-','linewidth',3);
subplot(224)
plot(T_list(1:sizU),U_list(2,:),'r-','linewidth',3);

U1_list1 = [];
U1_list2 = [];
t1_list = [];
U2_list1 = [];
U2_list2 = [];
t2_list = [];
U3_list1 = [];
U3_list2 = [];
t3_list = [];
U4_list1 = [];
U4_list2 = [];
t4_list = [];

for i = 1:step
   x = X_list(:,i);
   u = U_list(:,i);
   t = T_list(i)
   
   subplot(221);
   % throttle; bnd vel
   hold on;
%    plot(t,u(1),'.r', 'markersize',15);
   u1 = preXU_bnd.slice([1,2,3,5],[x(1:3);u(2)]);
   if u1.Num >= 1
       [v1,v2] = find_V(u1);
       U1_list1 = [U1_list1, v1];
       U1_list2 = [U1_list2, v2];
       t1_list = [t1_list,t];
   elseif ~isempty(t1_list)
       hold on;
       shade(t1_list,U1_list1,U1_list2);
       plot(t1_list,U1_list1,'-b','linewidth',3);
       plot(t1_list,U1_list2,'-b','linewidth',3);
       U1_list1 = [];
       U1_list2 = [];
       t1_list = [];
   end
   subplot(223);
   % steering; bnd vel
   hold on;   
%    plot(t,u(2),'.r', 'markersize',15);
   u2 = preXU_bnd.slice([1,2,3,4],[x(1:3);u(1)]);
   if u2.Num >= 1
       [v1,v2] = find_V(u2);
       U2_list1 = [U2_list1, v1];
       U2_list2 = [U2_list2, v2];
       t2_list = [t2_list,t];
   elseif ~isempty(t2_list)
       hold on;
       shade(t2_list,U2_list1,U2_list2);
       plot(t2_list,U2_list1,'-b','linewidth',3);
       plot(t2_list,U2_list2,'-b','linewidth',3);
       U2_list1 = [];
       U2_list2 = [];
       t2_list = [];
   end
   subplot(222);
   % throttle; cau
   hold on;
%    plot(t,u(1),'.r', 'markersize',15);
   u3 = preXU.slice([1,2,3,4,6],[x;u(2)]);
   if u3.Num >= 1
       [v1,v2] = find_V(u3);
       U3_list1 = [U3_list1, v1];
       U3_list2 = [U3_list2, v2];
       t3_list = [t3_list,t];
   elseif u1.Num >= 1
       [v1,v2] = find_V(u1);
       U3_list1 = [U3_list1, v1];
       U3_list2 = [U3_list2, v2];
       t3_list = [t3_list,t];
   elseif ~isempty(t3_list)
       hold on;
       shade(t3_list,U3_list1,U3_list2);
       plot(t3_list,U3_list1,'-b','linewidth',3);
       plot(t3_list,U3_list2,'-b','linewidth',3);
       U3_list1 = [];
       U3_list2 = [];
       t3_list = [];
   end
   subplot(224);
   % throttle; cau
   hold on;
%    plot(t,u(2),'.r', 'markersize',15);
   u4 = preXU.slice([1,2,3,4,5],[x;u(1)]);
   if u4.Num >= 1
       [v1,v2] = find_V(u4);
       U4_list1 = [U4_list1, v1];
       U4_list2 = [U4_list2, v2];
       t4_list = [t4_list,t];
   elseif u2.Num >= 1
       [v1,v2] = find_V(u2);
       U4_list1 = [U4_list1, v1];
       U4_list2 = [U4_list2, v2];
       t4_list = [t4_list,t];
   elseif ~isempty(t4_list)
       hold on;
       shade(t4_list,U4_list1,U4_list2);
       plot(t4_list,U4_list1,'-b','linewidth',3);
       plot(t4_list,U4_list2,'-b','linewidth',3);
       U4_list1 = [];
       U4_list2 = [];
       t4_list = [];
   end
   drawnow;
end
%%
subplot(221)
  hold on;
  shade(t1_list,U1_list1,U1_list2);
       plot(t1_list,U1_list1,'-b','linewidth',3);
       plot(t1_list,U1_list2,'-b','linewidth',3);
       title("throttle/bounded velocity inv set", 'interpreter','latex')
       xlabel("$t$", 'interpreter','latex');
       ylabel("$a_{e,x}$", 'interpreter','latex');
       set(gca,'fontsize',12);
subplot(223)
  hold on;
  shade(t2_list,U2_list1,U2_list2);
       plot(t2_list,U2_list1,'-b','linewidth',3);
       plot(t2_list,U2_list2,'-b','linewidth',3);   
       title("steering/bounded velocity inv set", 'interpreter','latex')
       xlabel("$t$", 'interpreter','latex');
       ylabel("$v_{e,y}$", 'interpreter','latex');
       set(gca,'fontsize',12);
subplot(222)
  hold on;
  shade(t3_list,U3_list1,U3_list2);
       plot(t3_list,U3_list1,'-b','linewidth',3);
       plot(t3_list,U3_list2,'-b','linewidth',3);
       title("throttle/aggressive driver inv set", 'interpreter','latex')
       xlabel("$t$", 'interpreter','latex');
       ylabel("$a_{e,x}$", 'interpreter','latex');
       set(gca,'fontsize',12);
subplot(224)
  hold on;
  shade(t4_list,U4_list1,U4_list2);
       plot(t4_list,U4_list1,'-b','linewidth',3);
       plot(t4_list,U4_list2,'-b','linewidth',3);
       title("steering/aggressive driver inv set", 'interpreter','latex')
       xlabel("$t$", 'interpreter','latex');
       ylabel("$v_{e,x}$", 'interpreter','latex');
       set(gca,'fontsize',12);

%%
function [v1,v2] = find_V(u)
    
    u.reduce();
    if u.Num == 1
        sub_u = u.Set(1);
        V = sub_u.V;
        v1 = min(V);
        v2 = max(V);
        return;
    end
    v1 = inf;
    v2 = -inf;
    for i = 1:u.Num
        sub_u = u.Set(i);
        V = sub_u.V;
        v1 = min(min(V),v1);
        v2 = max(max(V),v2);
    end
end

function shade(t_list,U_list1,U_list2)
    num = length(t_list);
    for i = 1:num-1
        t1 = t_list(i);

        t2 = t_list(i+1);
        v1_1 = U_list1(i);
        v1_2 = U_list1(i+1);
        v2_1 = U_list2(i);
        v2_2 = U_list2(i+1);
        V = [t1 v1_1;t1 v2_1;t2 v2_2;t2 v1_2];
        F = [1 2 3 4];
        patch('Faces',F,'Vertices',V,'FaceColor','b','EdgeColor','none','FaceAlpha',.1);
    end
end