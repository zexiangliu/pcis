function [ dyn0 ] = get_dyn_bdd_vel()
% get_dyn_bdd_vel.m for two car scenario (one front car; one incoming car)

%% Constants

con = constants_tri_two_car();
n_u = 2;
n_x = 4;

%% Algorithm

A = [	1-con.f1*con.dt 0 0 0;
	0 1 0 0;
	-con.dt 0 1 0;
	-con.dt 0 0 1];

B = [ eye(n_u)*con.dt ; zeros(n_x-n_u,n_u) ];
Bw = [ eye(n_x)*con.dt ];
Bw(:,4) = -Bw(:,4);

F = zeros(n_x,1);

XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

Ad = {zeros(4),zeros(4),zeros(4),zeros(4)};
Fd = { Bw(:,1) , Bw(:,2) , Bw(:,3), Bw(:,4) };

D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.vL_min,con.vL_min],'ub',[con.dmax_ACC,con.dmax_LK,con.vL_max,con.vL_max]);

dyn0 = Dyn(A,F,B,XU,...
	  {},{},Polyhedron(), ...
	  Ad, Fd, D);
