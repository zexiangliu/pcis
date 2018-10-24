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
% Time discretization
dt = 0.25;

%Gains taken from Yueying's Constants Document
K_ann = 5;
K_cau = 0.1*[ 2 3 0 ];
K_cau1 = K_cau(1);
K_cau2 = K_cau(2);
K_cau3 = K_cau(3);

vL_des = 25;

intent = 'cautious';

%% Define Dynamics
% Parameters when the annoying lead car is in effect.

A_ann =[    1   0 0       0;
            0   1 0       0;
            -dt 0 1       dt;
            0   0 -K_ann*dt 1];
B_ann = [   dt 0 ;
            0  dt;
            0  0;
            0  0];
Bw_ann = [ 0; 0; 0; dt ];
        
F_ann = zeros(size(A_ann,1),1);
        
% Parameters when the cautious lead driver is in effect.
A_cau =[1   0       0           0;
        0   1       0           0;
        -dt 0       1           dt;
        0   -K_cau2 -K_cau3*dt  1-K_cau1*dt];

B_cau = B_ann;
Bw_cau = Bw_ann;

F_cau = [0;0;0; K_cau1* vL_des * dt];

% State Constraints
ye_max = 1.6;
ye_ll_max = 1.6+3.2; %Left Lane Boundary
ye_min = -ye_max;

vex_max = vL_des*2;
vex_min = -vL_des/4;

%No h_max
h_min1 = 4;

vL_max = vex_max;
vL_min = 0;

%Input Constraints
u1_max = 9.8/2; %Acceleration of the ego vehicle's maximum value
u1_min = -9.8/2;

u2_max = 3; %Maximum Lateral Speed of the ego vehicle
u2_min = -u2_max;

%%%%%%%%%%%%%%%%%%%%%%%
%% Creating Safe Set %%
%%%%%%%%%%%%%%%%%%%%%%%

switch intent
    case 'annoying'
        A = A_ann;
        B = B_ann;
        Bw = Bw_ann;
        F = F_ann;
    case 'cautious'
        A = A_cau;
        B = B_cau;
        Bw = Bw_cau;
        F = F_cau;
    otherwise
        A = nan;
        B = nan;
        Bw = nan;
        F = nan;
        error(['Unrecognized Intention provided: ' intent]);
end

n = size(A,1);
m = size(B,2);
p = size(Bw,2);

X1 = Polyhedron('UB', [vex_max;     ye_ll_max;  300 ;       vL_max],...
                'LB', [vex_min;     ye_min;     h_min1;     vL_min]);
X2 = Polyhedron('UB', [vex_max;     ye_ll_max;  300 ;       vL_max],...
                'LB', [vex_min;     ye_max;     -300;       vL_min]);
X3 = Polyhedron('UB', [vex_max;     ye_ll_max;  -h_min1 ;       vL_max],...
                'LB', [vex_min;     ye_min;     -300;     vL_min]);
% Safe set 
S = PolyUnion([X1 X2 X3]); 

% cinv set
G = X2;

clf;hold on;
plot(X1.projection([2 3]));
plot(X3.projection([2 3]));            
plot(X2.projection([2 3]));
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
1;

XU = Polyhedron('UB', [ vex_max; ye_max; 300; vL_max ;u1_max;u2_max], ...
                'LB', [ vex_min; ye_min; h_min1; vL_min ; u1_min;u2_min]);

XD = Polyhedron('UB', [zeros(n,1);u1_max;u2_max],...
                'LB', [zeros(n,1);u1_min;u2_min]);

XD_mat = [zeros(p,n) eye(p) u1_max ; zeros(p,n) -eye(p) u1_min];

XD2 = {};
for i = 1:size(XD_mat,1)
    XD2{i} = XD_mat(i,:);
end

XW = Polyhedron('V',[u1_max;u1_min]);

d2 = Dyn(A, F, B, XU, ...
    [],[],[], ... %Ignoring anything with measurable disturbance
    {zeros(n)},{Bw},XW);

% reach
rho = 1e-6;
Xr = stay_invariant(d2, S, G, rho, 1);

hold on;
plot(G, 'color', 'g')
1;
