% takeover_expand.m
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
[dyn_a , dyn_c] = get_takeover_pwd_7regions();
% [dyn_a_dual , dyn_c_dual] = get_takeover_pwd_dual();
dyn_conserv = get_dyn_bdd_vel();
% [dyn_a_nn, dyn_c_nn] = get_takeover_pwd();
mptopt('lpsolver', 'GUROBI', 'qpsolver', 'GUROBI');

%% Select Intention to Use for Invariant Set Growth

dyn_opt = 3;
% 1 = dyn_a: Aggressive or Annoying Piecewise Affine Dynamics
% 2 = dyn_c: Cautious Piecewise Affine Dynamics
% 3 = dyn_conservative: Affine Dynamics with 3 states and the assumption that lead vehicle can arbitrarily choose its velocity from a bounded set.
% 4 = dyn_a, dual: Find the set of states for which a disturbance exists that will cause the system to be in the dual space (unsafe states)

%% Create Safe Set and Small Invariant Set
h_max = 4000;
vl_max = Inf;
X1 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;      vl_max],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -vl_max]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;     vl_max],...
                'LB', [con.v_min;   -con.y_min;     -h_max;    -vl_max]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    vl_max],...
                'LB', [con.v_min;   con.y_min;      -h_max;    -vl_max]);
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

% cinv set
% load CIS_bnd.mat
% C = lift_inv(CIS_bnd);


max_iter = 2;

% reach
rhoPre = 0; %1e-6;
switch dyn_opt
    case 1
        Xr = expand(dyn_a, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    case 2
        Xr = expand(dyn_c, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    case 3
        X1 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                        'LB', [con.v_min;   con.y_min;      con.h_min]);
        X2 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                        'LB', [con.v_min;   -con.y_min;     -Inf]);
        X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
                        'LB', [con.v_min;   con.y_min;      -Inf]);
        S = PolyUnion([X1 X2 X3]);
        C = X2;

        Xr = dyn_conserv.stay_invariant(S, C, rhoPre, 1 );
    case 4
        C = PolyUnion( [Polyhedron('lb',[con.v_min -Inf -3000 -Inf],'ub',[con.v_max, -0.9, 3000, Inf])] );
        S = PolyUnion([Polyhedron('lb',-Inf(1,4), 'ub',Inf(1,4) )]);
        Xr = dyn_a_dual.expand(S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    case 5
        seed = load("seed.mat");
        C = seed.C;
        Xr = expand(dyn_a, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    case 6 
        Xr = expand(dyn_a_nn, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    otherwise
        error(['Unrecognized dyn_opt value: ' num2str(dyn_opt) ])
end

%% Plotting
if Xr.Dim == 4
    figure;
    for ind = 1:Xr.Num
        subplot(221);hold on
        plot(Xr.Set(ind).slice([1 4], [25 25]));
        set(gca,'Xdir','reverse','Ydir','reverse')
        axis([-1 3 -50 50]);
        xlabel('ye'); ylabel('h');
        title('vEgo = 25 m/s, vLead = 25 m/s')

        subplot(222);hold on
        plot(Xr.Set(ind).slice([1 4], [30 20]));
        %plot(Xr{i}(end).slice([1 4], [30 20]));
        set(gca,'Xdir','reverse','Ydir','reverse')
        axis([-1 3 -50 50]);
        xlabel('ye'); ylabel('h');
        title('vEgo = 30 m/s, vLead = 20 m/s')

        subplot(223);hold on
        plot(Xr.Set(ind).slice([1 4], [16 25]));
        %plot(Xr{i}(end).slice([1 4], [16 25]));
        set(gca,'Xdir','reverse','Ydir','reverse')
        axis([-1 3 -50 50]);
        xlabel('ye'); ylabel('h');
        title('vEgo = 16 m/s, vLead = 25 m/s')

        subplot(224);hold on
        plot(Xr.Set(ind).slice([1 4], [25 0]));
        %plot(Xr{i}(end).slice([1 4], [25 0]));
        set(gca,'Xdir','reverse','Ydir','reverse')
        axis([-1 3 -50 50]);
        xlabel('ye'); ylabel('h');
        title('vEgo = 25 m/s, vLead = 0 m/s');
    end
elseif Xr.Dim == 3
    S1 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
                    'LB', [con.v_min;   con.y_min;      con.h_min]);
    S2 = Polyhedron('UB', [con.v_max;   con.y_max;      50],...
                    'LB', [con.v_min;   -con.y_min;     -50]);
    S3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
                    'LB', [con.v_min;   con.y_min;      -50]);

    figure;
    subplot(221); hold on;
    temp_intersx = IntersectPolyUnion(Xr,PolyUnion([S1 S2 S3]));
    plot(temp_intersx.slice([1],25),'color','red')
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 3 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s')

    subplot(222); hold on;
    plot(temp_intersx.slice([1],30),'color','red')
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 3 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s')

    subplot(223); hold on;
    plot(temp_intersx.slice([1],16),'color','red')
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 3 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s')

    subplot(224); hold on;
    plot(temp_intersx.slice([1],20),'color','red')
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 3 -50 50]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 20 m/s')
else
    error(['The resulting Xr has an unexpected dimension: ' num2str(Xr.Dim)])
end

    
