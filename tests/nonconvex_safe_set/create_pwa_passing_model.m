function [ pwd0 ] = create_pwa_passing_model( varargin )
% Description:
% 
%	This model was built according to the specifications in the 'Piecewise 
%	Affine Dynamics for "Saturating" Disturbances in ACC Model' section of
%	[Read Only Link: https://v2.overleaf.com/read/qkyggdvzxdfk]
%
% Usage:
%	pwd0 = create_pwa_passing_model( intention )
%	pwd0 = create_pwa_passing_model( intention, 'show_regions' )

%% Input Processing

intention = varargin{1};

%Default Values for some other flags
plot_regions = false;

if nargin >= 2
	for ind = 2:nargin
		switch varargin{ind}
			case 'show_regions'
				plot_regions = true;
			otherwise
				error(['Unrecognized Flag: ' varargin{ind} ])
		end
	end
end


%% Input Processing

switch intention
	case {'annoying'}
		disp(['Selected ''' intention ''' as intention.'])
	otherwise
		error(['Unrecognized intention model: ' intention ])
end

%% Constants
con = constants_tri;
nx = 4;

%% Create Domain

%Recall the state is 4 dimensional, with:
%   state = [v_e,x]
%           [y_e  ]
%           [h    ]
%           [v_L,x]

dom = Polyhedron('lb',-inf(1,nx),'ub',inf(1,nx) );


%% Create Regions for Each Dynamics to Use
eta_A = 0.5*con.aL_max;

reg1 = Polyhedron('A',[ 0 0 -con.K_ann 0; 0 0 con.K_ann 0	],'b',[con.aL_max - eta_A;-con.aL_min - eta_A]); %equivalent to conditions 9 and 10
reg2 = Polyhedron('A',[ 0 0 -con.K_ann 0],'b',[con.aL_min + eta_A]);
reg3 = Polyhedron('A',[ 0 0 con.K_ann 0],'b',[-con.aL_max + eta_A]);

if plot_regions
	figure;
	subplot(3,1,1)
	plot(reg1.projection([2 3]))

	subplot(3,1,2)
	plot(reg2.projection([2 3]))

	subplot(3,1,3)
	plot(reg3.projection([2 3]))
end

reg_list = {reg1,reg2,reg3};

%% Create Dynamics

%Create A,B,B_w, and F matrices for the continuous time model
%
% x'(t) = Ax(t) + Bu(t) + B_w w(t) + F
%

A = [	-con.f1 0	0 	0;
		0		0	0	0;
		-1		0	0	1;
		0		0	0	0];

A_ka = A;
A_ka(4,3) = -con.K_ann;

B = [eye(2);zeros(2)];
B_w = [zeros(3,1);1];

F = zeros(nx,1);
F_sat2 = [zeros(3,1); con.aL_min + eta_A ]; %Offset Term Created By Saturation in region 2
F_sat3 = [zeros(3,1); con.aL_max - eta_A ]; %Offset Term Created By Saturation in region 3

%============================
%Create Dynamics for Region 1
%============================

XU = Polyhedron('A', [zeros(1,nx) 1 0;
                      zeros(1,nx) 0 1;
                      zeros(1,nx) -1 0;
                      zeros(1,nx) 0 -1],...
                'b', [con.umax_ACC;
                      con.umax_LK;
                      -con.umin_ACC;
                      -con.umin_LK]);

D = Polyhedron('UB', [eta_A] , 'LB', [-eta_A] );

%Discretize

sysC = ss(A_ka, B, [], []);
sysD = c2d(sysC, con.dt);

sys_matrices.A = sysD.A;
sys_matrices.B = sysD.B;

sysC = ss(A_ka, F, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.F = sysD.B;

sysC = ss(A_ka, B_w, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.B_w = sysD.B;

Ad = {zeros(nx)};%{zeros(nx), zeros(nx)};% zeros(nx)};           
Fd = {sys_matrices.B_w};%{B(:,1), B(:,2)};%, zeros(nx,1)};


d1 = Dyn(	sys_matrices.A, sys_matrices.F, sys_matrices.B, XU, ...
			[],[],[], ... %Ignoring anything with measurable disturbance)
			Ad,Fd,D);

%============================
%Create Dynamics for Region 2
%============================

clear sys_matrices;

% XU = Polyhedron('A', [zeros(1,nx) 1 0;
%                       zeros(1,nx) 0 1;
%                       zeros(1,nx) -1 0;
%                       zeros(1,nx) 0 -1],...
%                 'b', [u1_max;
%                       u2_max;
%                       -u1_min;
%                       -u2_min]);

% D = Polyhedron('UB', [eta_A] , 'LB', [-eta_A] );

%Discretize

sysC = ss(A, B, [], []);
sysD = c2d(sysC, con.dt);

sys_matrices.A = sysD.A;
sys_matrices.B = sysD.B;

sysC = ss(A, F_sat2, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.F = sysD.B;

sysC = ss(A, B_w, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.B_w = sysD.B;

Ad = {zeros(nx)};%{zeros(nx), zeros(nx)};% zeros(nx)};           
Fd = {sys_matrices.B_w};%{B(:,1), B(:,2)};%, zeros(nx,1)};


d2 = Dyn(	sys_matrices.A, sys_matrices.F, sys_matrices.B, XU, ...
			[],[],[], ... %Ignoring anything with measurable disturbance)
			Ad,Fd,D);

%============================
%Create Dynamics for Region 3
%============================

clear sys_matrices;

% XU = Polyhedron('A', [zeros(1,nx) 1 0;
%                       zeros(1,nx) 0 1;
%                       zeros(1,nx) -1 0;
%                       zeros(1,nx) 0 -1],...
%                 'b', [u1_max;
%                       u2_max;
%                       -u1_min;
%                       -u2_min]);

% D = Polyhedron('UB', [eta_A] , 'LB', [-eta_A] );

%Discretize

sysC = ss(A, B, [], []);
sysD = c2d(sysC, con.dt);

sys_matrices.A = sysD.A;
sys_matrices.B = sysD.B;

sysC = ss(A, F_sat3, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.F = sysD.B;

sysC = ss(A, B_w, [], []);
sysD = c2d(sysC, con.dt);
sys_matrices.B_w = sysD.B;

Ad = {zeros(nx)};%{zeros(nx), zeros(nx)};% zeros(nx)};           
Fd = {sys_matrices.B_w};%{B(:,1), B(:,2)};%, zeros(nx,1)};


d3 = Dyn(	sys_matrices.A, sys_matrices.F, sys_matrices.B, XU, ...
			[],[],[], ... %Ignoring anything with measurable disturbance)
			Ad,Fd,D);

dyn_list = {d1,d2,d3};

%% Final Result

pwd0 = PwDyn(dom,reg_list,dyn_list);




