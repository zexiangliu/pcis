% takeover_main.m
% Takeover Scenario for TRI

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
con = constants_tri;
% Get Dynamics
dyn_unknown = get_dyn_bdd_vel();
[dyn_a , dyn_c] = get_takeover_pwd();

% Find a conservative cinv set valid for unknown intention (bounded vel)
X1 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf],...
                'LB', [con.v_min;   -con.y_min;     -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min],...
                'LB', [con.v_min;   con.y_min;      -Inf]);
S = PolyUnion([X1 X3]);
C = X2;

C_unknown = dyn_unknown.stay_invariant2(S, C, 0, 0 );


% Expand this set with intentions
h_max = Inf;
vl_max = Inf;
X1 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;      vl_max],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -vl_max]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;     vl_max],...
                'LB', [con.v_min;   -con.y_min;     -h_max;    -vl_max]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    vl_max],...
                'LB', [con.v_min;   con.y_min;      -h_max;    -vl_max]);

% Safe set 
S = PolyUnion([X1 X2 X3]); 

% cinv set
C = PolyUnion;
for c = 1:C_unknown.Num
    C.add(Polyhedron('A', [C_unknown.Set(c).A, zeros(size(C_unknown.Set(c).A,1),1); 0 0 0 1; 0 0 0 -1],...
                     'b', [C_unknown.Set(c).b; con.vL_max; -con.vL_min] ));
end

max_iter = 1;
rhoPre = 0; %1e-6;

%%
C_a = expand(dyn_a, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);
%%
% C_c = expand(dyn_c, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);