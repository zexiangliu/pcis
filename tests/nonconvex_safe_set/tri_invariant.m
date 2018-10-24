% ACC example for TRI
% x = [vEgo; yEgo; h; vlead]
% u = [aEgo; dyEgo]
% d = [aLead];
% tri_overtake.m
% Compute an invariant set for a triple integ

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
intent = 'cautious' %'annoying'; % 'cautious';
con = constants_tri;


%% Define Dynamics
% Contunious dynamics
switch intent
    case 'annoying'
        A = [ -con.f1   0       0       0;
                0       0       0       0;
                -1      0       0       1;
                0       0   con.K_ann   0 ];
        F = zeros(size(A,1),1);
    case 'cautious'
        A = [ -con.f1   0       0       0;
                 0      0       0       0;
                -1      0       0       1;
                0 -con.K2_cau con.K3_cau -con.K1_cau];
        F = [0;0;0; con.K1_cau* con.vL_des];
    otherwise
        error(['Unrecognized Intention provided: ' intent]);
end
   
B = [1 0;
     0 1; 
     0 0; 
     0 0];    
 
% discretize
sysC = ss(A, B, [], []);
sysD = c2d(sysC, con.dt);
B = sysD.B;
sysC = ss(A, F, [], []);
sysD = c2d(sysC, con.dt);
A = sysD.A;
F = sysD.B;

%%%%%%%%%%%%%%%%%%%%%%%
%% Creating Safe Set %%
%%%%%%%%%%%%%%%%%%%%%%%

nx = size(A,1);
nu = size(B,2);
% nd = size(D,2);

% X1 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf;           Inf],...
%                 'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
% X2 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf;           Inf],...
%                 'LB', [con.v_min;   -con.y_min;     -Inf;          -Inf]);
% X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
%                 'LB', [con.v_min;   con.y_min;      -Inf;          -Inf]);
            
            
X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
                'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
                'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);
% X4 = Polyhedron()
            
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

XU = Polyhedron('A', [zeros(1,nx) 1 0;
                      zeros(1,nx) 0 1;
                      zeros(1,nx) -1 0;
                      zeros(1,nx) 0 -1],...
                'b', [con.umax_ACC;
                      con.umax_LK;
                      -con.umin_ACC;
                      -con.umin_LK]);
1;
D = Polyhedron('UB', [con.dmax_ACC;con.dmax_LK], ...%; con.dLmax],...
               'LB', [con.dmin_ACC;con.dmin_LK]);% con.dLmin]);

Ad = {zeros(nx), zeros(nx)};% zeros(nx)};           
Fd = {B(:,1), B(:,2)};%, zeros(nx,1)};


% d = Dyn(A, F, B, XU, ...
%     [],[],[], ... %Ignoring anything with measurable disturbance
%     [],[],[], ... %Ignoring anything with nonmeasurable disturbance
%     [],[], ... %Ignoring state-dependent
%     Bw,XD2);
% d = Dyn(A, F, B, XU, Ap, Fp, P, Ad, Fd, D, Ev, XV_V, Ew, XW_V)
dyn = Dyn(A, F, B, XU, ...
    [],[],[], ... %Ignoring anything with measurable disturbance
    Ad,Fd,D);

% reach
rhoPre = 1e-6;
Xr = expand(dyn, S, C, rhoPre,'debug');

figure;
hold on;
for i_xr = 1:length(Xr)
  plot(Xr{i_xr}.projection([2 3]), 'color', 'g')
end
