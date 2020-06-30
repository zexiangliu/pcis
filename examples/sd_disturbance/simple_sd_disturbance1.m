%simple_sd_disturbance1.m
%Description:
%	Tests a very simple example of using Dyn() objects that use state-dependent disturbances.

clear all;
close all;
clc;

%% Create a Simple Two-Dimensional, Completely Controlled System %%

% x_+ = (A1 + ∑d_i Ad_list{i}) x + B1 u + ∑d_i(k) Fd_list{i} 

A1 = eye(2);
B1 = zeros(2,1);

dim_x = size(A1,1);
dim_u = size(B1,2);

F1 = zeros(dim_x,1); %Affine term in dynamics
domain_bound_x = 10;
XU = Polyhedron('lb',[-domain_bound_x*ones(1,dim_x),-ones(1,dim_u)], ...
				'ub',[domain_bound_x*ones(1,dim_x),ones(1,dim_u)]);

Fd1 = eye(2);
dim_d = size(Fd1,2);

Ad_list = { zeros(dim_x) , zeros(dim_x) }; %There must be as many dim_x-by-dim_x matrices in Ad as there are dimensions of d.
Fd_list = {Fd1(:,1),Fd1(:,2)};

d_bound = 1;
%D1 = Polyhedron('A',[eye(dim_d);-eye(dim_d)],'b',[d_bound;0;0;d_bound]);
D1 = Polyhedron('lb',[d_bound;-d_bound],'ub',[d_bound;-d_bound]);

%This disturbance set:
%	- Should have a single value in it
%	- Represents a wind in the Southeastern direction

dyn1 = Dyn( A1 , F1 , B1 , XU , ...
			{}, {}, Polyhedron(), ...
            Ad_list,Fd_list, D1 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Compute the Pre of a Simple Set %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rho1 = 0;
SimpleSet = Polyhedron('lb',-ones(1,dim_x),'ub',ones(1,dim_x));

pre_ss1 = dyn1.pre( SimpleSet );

alpha1 = 0.4;

figure;
hold on;
plot(SimpleSet,'Alpha',alpha1)
plot(pre_ss1,'Color','Blue','Alpha',alpha1,'LineStyle',':')

axis1 = [-5,5,-5,5];
axis(axis1)

title('CPre of a Set with Zero Input + Singleton Disturbance Set')

%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2a. Create System 2 %%
%%%%%%%%%%%%%%%%%%%%%%%%%

%System Properties:
%	- Unmeasured, State-Independent Disturbance
%	- No Control Input
%	- Disturbance Set if Full-Dimensional (2 Dimensions and not singleton)

% x_+ = (A2 + ∑d_i Ad_list{i}) x + B2 u + ∑d_i(k) Fd_list{i} 

A2 = eye(2);
B2 = zeros(2,1);

dim_x = size(A2,1);
dim_u = size(B2,2);

F2 = zeros(dim_x,1); %Affine term in dynamics
domain_bound_x = 10;
XU = Polyhedron('lb',[-domain_bound_x*ones(1,dim_x),-ones(1,dim_u)], ...
				'ub',[domain_bound_x*ones(1,dim_x),ones(1,dim_u)]);

Fd2 = eye(2);
dim_d = size(Fd2,2);

Ad_list = { zeros(dim_x) , zeros(dim_x) }; %There must be as many dim_x-by-dim_x matrices in Ad as there are dimensions of d.
Fd_list = {Fd2(:,1),Fd2(:,2)};

d_bound = 1;
D2 = Polyhedron('lb',[0,-d_bound],'ub',[d_bound,0]);

%This disturbance set:
%	- Should have a single value in it
%	- Represents a wind in the Southeastern direction

dyn2 = Dyn( A2 , F2 , B2 , XU , ...
			{}, {}, Polyhedron(), ...
            Ad_list,Fd_list, D2 );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2b. Compute the Pre of a Simple Set %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SimpleSet = Polyhedron('lb',-ones(1,dim_x),'ub',ones(1,dim_x));

pre_ss2 = dyn2.pre( SimpleSet );

alpha2 = alpha1;

figure;
hold on;
plot(SimpleSet,'Alpha',alpha2)
plot(pre_ss2,'Color','Blue','Alpha',alpha2,'LineStyle',':')

axis2 = axis1;
axis(axis2)

title('CPre of a Set with Zero Input + Full-Dimensional Disturbance Set')

%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3a. Create System 3 %%
%%%%%%%%%%%%%%%%%%%%%%%%%

%System Properties:
%	- Unmeasured, State-Dependent Disturbance
%	- No Control Input
%	- Disturbance Set if Full-Dimensional (2 Dimensions and not singleton)

% 	x(k+1) = A3 x(k) + B3 u(k) + Ew3 w(k)
% 	where
%		w(k) ∈ conv_i(XPW_V{i} [x(k); p(k); 1]) non-measurable state-dependent disturbance

A3 = eye(2);
B3 = zeros(2,1);

dim_x = size(A3,1);
dim_u = size(B3,2);

F3 = zeros(dim_x,1); %Affine term in dynamics
domain_bound_x = 10;
XU = Polyhedron('lb',[-domain_bound_x*ones(1,dim_x),-ones(1,dim_u)], ...
				'ub',[domain_bound_x*ones(1,dim_x),ones(1,dim_u)]);

Ew3 = eye(2);
dim_w = size(Ew3,2);

w_bound = 1;
%Define a disturbance set similar to D2

%Define the first disturbance w(1), so that its range is:
%	- [0,0] when x(1) = 1
%	- [0,d_bound] when x(1) = -1
%
%Use simple linear interpolation to define this time-varying bound on w(1)
%	w(1) = m1 * x(1) + b1
%
%This linear form will be important when we start defining XPW_V. The matrices
%in that variable define the "bounds" on the disturbance as well:
%	bound1 = XPW_V{1} * [ x(1) , x(2) , [] , 1 ]'
%
%Note that I put an extra empty matrix to represent the place where the measurable disturbance
%would normally go. Because we have none in this example, we will not have anything there, so in reality
%	bound1 = XPW_V{1} * [ x(1) , x(2) , 1 ]'
%
%Because we've defined our linear rule, bound1 and bound2 for JUST THE w(1) disturbance can be interpreted as:
%	bound1 = [0] = zeros(1,3) * [ x(1) , x(2) , 1 ]'
%	bound2 = m1*x(1) + b1 = [ m1 , 0 , b1 ] * [ x(1) , x(2) , 1 ]';

lin_params = [ 1 , 1; -1,1 ]^(-1) * [0;w_bound];
m1 = lin_params(1);
b1 = lin_params(2);

%Define the second disturbance w(2), so that its range is:
%	- [0,0] when x(2) = -1
%	- [0,-d_bound] when x(2) = 1
%
%Use simple linear interpolation to define this time-varying bound on w(1)
%	w(2) = m2 * x(2) + b1
%

lin_params = [ -1 , 1; 1,1 ]^(-1) * [0;-w_bound];
m2 = lin_params(1);
b2 = lin_params(2);

XPW_V = {
	%If you uncomment these lines, then you can recreate #2's plot

	%zeros(2,dim_x+1),
	%[ 1 , 0 , 1 ; zeros(1,dim_x+1) ],
	%[ zeros(1,dim_x+1) ; 0 , 1 , -1],
	%[ 1 , 0 , 1 ; 0 , 1 , -1 ]

	% zeros(2,dim_x+1),
	% 0.5*[ 1 , 0 , 1 ; zeros(1,dim_x+1) ],
	% 0.5*[ zeros(1,dim_x+1) ; 0 , 1 , -1],
	% 0.5*[ 1 , 0 , 1 ; 0 , 1 , -1 ]

	zeros(2,dim_x+1),
	[ m1 , 0 , b1 ; zeros(1,dim_x+1) ],
	[ zeros(1,dim_x+1) ; 0 , m2 , b2],
	[ m1 , 0 , b1 ; 0 , m2 , b2 ]

};

%This disturbance set:
%	- Should have a single value in it
%	- Represents a wind in the Southeastern direction

dyn3 = Dyn( A3 , F3 , B3 , XU , ...
			{}, {}, Polyhedron(), ...
			{}, {}, Polyhedron(), ...
			[],Polyhedron(),...
            Ew3, XPW_V );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2b. Compute the Pre of a Simple Set %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

SimpleSet = Polyhedron('lb',-ones(1,dim_x),'ub',ones(1,dim_x));

pre_ss3 = dyn3.pre( SimpleSet );

alpha3 = alpha1;

figure;
hold on;
plot(SimpleSet,'Alpha',alpha3)
plot(pre_ss3,'Color','Blue','Alpha',alpha3,'LineStyle',':')

axis3 = axis1;
axis(axis3)

title('CPre of a Set with Zero Input + State-Dependent Disturbance Set')

