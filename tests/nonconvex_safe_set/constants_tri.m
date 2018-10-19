function [con] = constants_tri()

% Time discretization
con.dt = 0.25;

%Gains taken from Yueying's Constants Document
con.K_ann = 5;
con.K_cau = 0.1*[ 2 3 0 ];
con.K_cau1 = con.K_cau(1);
con.K_cau2 = con.K_cau(2);
con.K_cau3 = con.K_cau(3);

con.vL_des = 25;

con.intent = 'cautious';

con.f1 = 1;

% State Constraints
con.ye_max = 1.6;
con.ye_ll_max = 1.6+3.2; %Left Lane Boundary
con.ye_min = -con.ye_max;

con.vex_max = con.vL_des*2;
con.vex_min = -con.vL_des/4;

%No h_max
con.h_min = 4;
con.h_max = 300;

con.vL_max = con.vex_max;
con.vL_min = 0;

%Input Constraints
con.u1_max = 9.8/2; %Acceleration of the ego vehicle's maximum value
con.u1_min = -9.8/2;

con.u2_max = 3; %Maximum Lateral Speed of the ego vehicle
con.u2_min = -con.u2_max;

%Disturbance Constraints
con.etaI_max = 1;
con.etaI_min = -1;