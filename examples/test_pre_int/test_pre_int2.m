%test_pre_int2.m
%Overview:
%   Tests pre_int with 2 very important 2d examples.
%Summary:
%   In one example, the intersection of the XU polytopes should be empty
%   although the pre of each individual pwd is nonempty. (pwd3,pwd4)
%   In the other example, the intersection of the XU polytopes should be
%   nonempty and the pre of each individual pwd is nonempty. (pwd1,pwd2)
%

clear all; close all; clc;

%% Constants
n = 2;
XU = Polyhedron('lb',[-inf(1,n) -1],'ub',[inf(1,n) 1]);
d1 = Dyn(eye(n),zeros(n,1),[1;0], XU);
d2 = Dyn(eye(n),zeros(n,1),[0;1], XU);

X = Polyhedron('lb',-inf(1,n),'ub',inf(1,n));

pwd1 = PwDyn( X , {X}, {d1} );
pwd2 = PwDyn( X , {X}, {d2} );

%% Calculating Pres
rho = 10^(-3);
R = Polyhedron('lb',-ones(1,2), 'ub', ones(1,2) );
pre1 = pwd1.pre( R , rho );
pre2 = pwd2.pre( R , rho );

figure;
subplot(1,2,1)
plot(pre1)
axis([ -3 3 -3 3])

subplot(1,2,2)
plot(pre2)
axis([ -3 3 -3 3])

%% Calculating more interesting pre's.

d3 = Dyn(eye(n),-[-1;-2],[1;0], XU);
d4 = Dyn(eye(n),-[1;-2],[1;0], XU);
pwd3 = PwDyn( X , {X}, {d3} );
pwd4 = PwDyn( X , {X}, {d4} );

pre3 = pwd3.pre( R , rho );
pre4 = pwd4.pre( R , rho );

figure;
subplot(1,2,1)
plot(pre3)
axis([ -5 5 -5 5])

subplot(1,2,2)
plot(pre4)
axis([ -5 5 -5 5])

%% Calculate the Intersect pre
%Intersect Pre should be known, but let's make sure that it is being
%correctly found.

R = PolyUnion(R);
pre_isx1 = pre_int(pwd1, pwd2, R, rho);
pre_isx2 = pre_int(pwd3, pwd4, R, rho);

%pre_isx1 should be nonempty while pre_isx2 should be empty.

