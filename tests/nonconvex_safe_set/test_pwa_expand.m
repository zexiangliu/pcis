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

% % choose lead car's intention model
intent = 'annoying'; % 'cautious';
con = constants_tri;

% Get Dynamics
pwd1 = create_pwa_passing_model(intent);

%% Create Safe Set and Small Invariant Set

X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
                'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
                'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);

% Safe set 
S = PolyUnion([X1 X3]); 
% figure;clf;hold on
% for s=1:S.Num
%     plot(S.Set(s).projection([2 3]));
% end
% set(gca,'Xdir','reverse','Ydir','reverse');
% xlabel('y');
% ylabel('h');
    

% cinv set
C = X2;

% reach
rhoPre = 1e-6;
Xr = expand(pwd1, S, C, rhoPre,'plot_stuff');

figure;
hold on;
for i_cell = 1:length(Xr)
  for i_poly = 1:length(Xr{i_cell})
    plot(Xr{i_cell}(i_poly).projection([2 3]), 'color', 'green')
  end
end
