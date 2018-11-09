% test_pwa_expand.m
% Passing Scenario for TRI

% System matrices,
%   state = [v_e,x]
%           [y_e  ]
%           [h    ]
%           [v_L,x]
%
%   where - v_e,x is the ego car's velocity in the x direction
%         - y_e is the ego car's lateral displacement (where left is
%         positive, right is negative.)
%         - h is the distance between the lead car's back bumper and the
%         ego car's front bumber (positive indicates ego is following the
%         lead car)
%         - v_L,x is the lead car's velocity in the longitudinal or x
%         direction

%% Constants %%
clear;close all;clc;
con = constants_tri();
% Get Dynamics
[dyn_a , dyn_c] = get_takeover_pwd();
con.h_max = 3000;

%% Create Safe Set and Small Invariant Set

X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
                'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
                'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);

% Safe set 
S = PolyUnion([X1 X2 X3]); 
% figure;clf;hold on
% for s=1:S.Num
%     plot(S.Set(s).projection([2 3]));
% end
% set(gca,'Xdir','reverse','Ydir','reverse');
% xlabel('y');
% ylabel('h');

% cinv set
C = X2;

max_iter = 7;

% reach
rhoPre = 0; %1e-6;
Xr = expand(dyn_c, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);

%% Plotting
figure;
for ind = 1:Xr.Num
    subplot(221);hold on
    plot(Xr.Set(ind).slice([1 4], [25 25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 25 m/s')

    subplot(222);hold on
    plot(Xr.Set(ind).slice([1 4], [30 20]));
    %plot(Xr{i}(end).slice([1 4], [30 20]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s, vLead = 20 m/s')

    subplot(223);hold on
    plot(Xr.Set(ind).slice([1 4], [16 25]));
    %plot(Xr{i}(end).slice([1 4], [16 25]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s, vLead = 25 m/s')

    subplot(224);hold on
    plot(Xr.Set(ind).slice([1 4], [25 0]));
    %plot(Xr{i}(end).slice([1 4], [25 0]));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 0 m/s');
end
