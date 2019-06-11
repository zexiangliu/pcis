% takeover_expand.m
% Passing Scenario for TRI

% System matrices,
%   state = [v_e,x]
%           [y_e  ]
%           [h_1    ]
%           [h_2]
%
%   where - v_e,x is the ego car's velocity in the x direction
%         - y_e is the ego car's lateral displacement (where left is
%         positive, right is negative.)
%         - h_1 is the distance between the lead car's back bumper and the
%         ego car's front bumber (positive indicates ego is following the
%         lead car)
%         - h_2 is the distance between the incoming car and ego car

%% Constants %%
clear;close all;clc;
con = constants_tri_two_car();
% Get Dynamics
dyn2 = get_dyn_bdd_vel_two_car();
dyn1 = get_dyn_bdd_vel_one_car();

mptopt('lpsolver', 'GUROBI', 'qpsolver', 'GUROBI');

%% Create Safe Set and Small Invariant Set

h_max = inf;

% right lane, behind
X1 = Polyhedron('UB', [con.v_max;   con.y_mid;      h_max;      h_max],...
                'LB', [con.v_min;   con.y_min;      con.h_min;  -h_max]);
% right lane, front
X2 = Polyhedron('UB', [con.v_max;   con.y_mid;      -con.h_min; h_max],...
                'LB', [con.v_min;   con.y_min;      -h_max;    -h_max]);
% left lane, behind
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;     h_max],...
                'LB', [con.v_min;   con.y_mid;     -h_max;     con.h_min]);
% lefe lane, front
X4 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;     -con.h_min],...
                'LB', [con.v_min;   con.y_mid;     -h_max;     -h_max]);

% Safe set 
S = PolyUnion([X1 X2 X3 X4]); 
max_iter = 5;
%%

% compute the seed set
% X1_lower = Polyhedron('UB', [con.v_max;   con.y_mid;      h_max],...
%                 'LB', [con.v_min;   con.y_min;      con.h_min]);
% % C1 = dyn1.win_always_rho_var(X1_lower,@rho_var,1,1);
% C1 = dyn2.win_always_rho_var(X1,@rho_var,1,1);
C1 = dyn2.win_always(X1,0,1,1);
C2 = dyn2.win_always_rho_inv(X2,@rho_var,1,1);
%%
% C3 = dyn2.win_always_rho_inv(X3,@rho_var,1,1);
C4 = dyn2.win_always_rho_inv(X4,@rho_var,1,1);
%%
C = PolyUnion([C1,C2,C4]);
% save seed_set.mat C
%%
rhoPre = 1e-6;
dom = Polyhedron('UB',[5000, 5000, inf, inf],'LB', [-5000, -5000, -inf, -inf]);
dyn = PwDyn(dom,{dom},{dyn2});
CIS = expand(dyn, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
% reach
rhoPre = 0;
switch dyn_opt
    case 1
        CIS_ann = expand(dyn_a, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
    case 2
        CIS_cau = expand(dyn_c, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
end

    