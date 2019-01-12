function [ pwd_A , pwd_C ] = get_takeover_pwd( varargin )
%get_takeover_pwd This function returns the 2 peicewise dynamics defined in
%Yunus Sahin's document
%[https://umich.box.com/s/mf77npzwp13jiifvg72ee126g0x3psqa].
%   
%   Inputs:
%       h_lim:  A real number.
%               This is the limiting value of the domain of the pwd system in the "h" dimension.
%               i.e. h \in [ -h_lim, h_lim]
%
%   Output:
%       pwd_A: The peicewise affine dynamics for the ANNOYING driver.
%

%% Constants

%Maybe I should load some of these.
con = constants_tri;

if nargin == 0
  h_lim = Inf;
else
  h_lim = varargin{1};
end

%% Base Matrices

n_x = 4; %Dimension of the state space
n_u = 2; %Dimension of the input space

A = [ 1-con.f1*con.dt 0 0 0;
      0 1 0 0;
      -con.dt 0 1 con.dt;
      0 0 0 1-con.f1*con.dt];
  %%% unsure f_l

B = [eye(2)*con.dt; zeros(2)];

Bw = B;

%% Matrix Modifications for the Annoying driver.

%=============================
%Region 1, for Annoying Driver
%Region in which the feedback can saturate the upper VELOCITY bound, so
%its velocity can change arbitrarily in the feasible domain.
%Region in "front of the car". Constraint is implicit.

A_r1 = zeros(n_x);
A_r1(4,:) = -A(4,:);
F_r1 = zeros(n_x,1);

Bw_r1 = { Bw(:,1), Bw(:,2), zeros(n_x,1) };

%Define Polyhedral domain as Hx * x <= h_x
Hx_r1 = [ -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
          [ zeros(2,2) [1;-1] zeros(2,1) ] ]; 
hx_r1 = [-(con.vL_max - con.dLmax*con.dt);
          con.h_reaction;
          -con.h_reaction];
r1 = Polyhedron('A',Hx_r1,'b',hx_r1);

%Create State Dependent Disturbance
Ew_r1 = [zeros(3,1);con.dt];
XW_V_r1 = { [zeros(1,4),con.vL_min],[zeros(1,4),con.vL_max] };

%Region 1.5
Hx_r1_5 = [ 0, 0, -1, 0 ];
hx_r1_5 = [con.h_reaction];

r1_5 = Polyhedron('A',Hx_r1_5,'b',hx_r1_5);


%=============================
%Region 2, for Annoying Driver
%Region in which:
% - the feedback is not saturating the VELOCITY bound.
% - the feedback can saturate the lower bound on ACCELERATION bound.

A_r2 = zeros(n_x);
A_r2(4,:) = con.K_ann*con.dt;

F_r2 = zeros(n_x,1);
Bw_r2 = { Bw(:,1), Bw(:,2), zeros(n_x,1) };

Hx_r2 = [   con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
            -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ;
            con.K_ann ;
            con.K_ann ];
hx_r2 = [   con.vL_max - con.dLmax*con.dt ;
            -con.vL_min + con.dLmin*con.dt;
            con.h_reaction ;
            -con.h_reaction ;
            con.aL_min - con.dLmin;
            con.aL_max - con.dLmax];
r2 = Polyhedron('A',Hx_r2,'b',hx_r2);

%Create State Dependent Disturbance
Ew_r2 = [zeros(3,1);con.dt];
XW_V_r2 = { [zeros(1,4),con.aL_min],[con.K_ann,con.dLmax] };

%=============================
%Region 3, for Annoying Driver
%Region in which:
% - the feedback is not saturating the VELOCITY bound.
% - the feedback can saturate the upper bound on ACCELERATION bound.

A_r3 = zeros(4);
A_r3(4,:) = con.K_ann*con.dt;

F_r3 = zeros(n_x,1);
Bw_r3 = {Bw(:,1), Bw(:,2), zeros(n_x,1)};

Hx_r3 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
          -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
          [ zeros(2,2) [1;-1] zeros(2,1) ] ;
          -con.K_ann ;
          -con.K_ann ];
hx_r3 = [ con.vL_max - con.dLmax*con.dt ;
          -con.vL_min + con.dLmin*con.dt;
          con.h_reaction ;
          -con.h_reaction ;
          -(con.aL_min - con.dLmin);
          -(con.aL_max - con.dLmax)];
r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

%Create State Dependent
Ew_r3 = Ew_r2;
XW_V_r3 = { [con.K_ann,con.dLmin],[zeros(1,4),con.aL_max] };

%==========================================================
%Region 4, for Annoying Driver (Outside of Reaction Zone 1)
%Region in which:
% - the feedback is not saturating the VELOCITY bound.
% - the feedback is not saturating the ACCELERATION bound.

A_r4 = zeros(n_x);
A_r4(4,:) = con.K_ann*con.dt;

F_r4 = [ zeros(3,1) ; -con.K_des*[0; 0;0;con.vL_des] ];
Bw_r4 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

Hx_r4 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
          -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
          [ zeros(2,2) [1;-1] zeros(2,1) ] ;
          -con.K_ann ;
          con.K_ann];
hx_r4 = [ con.vL_max - con.dLmax*con.dt ;
          -con.vL_min + con.dLmin*con.dt;
          con.h_reaction ;
          -con.h_reaction ;
          -(con.aL_min - con.dLmin);
          con.aL_max - con.dLmax];
r4 = Polyhedron('A',Hx_r4,'b',hx_r4);
        
%==========================================================
%Region 5, for Annoying Driver (Outside of Reaction Zone 2)
%Region in which:
% - the feedback can saturate the lower VELOCITY bound
%, so its velocity can change arbitrarily in the feasible domain.
%Region in "front of the car". Constraint is implicit.

A_r5 = zeros(n_x);
A_r5(4,:) = -A(4,:);

F_r5 = F_r1;
Bw_r5 = Bw_r1;

Hx_r5 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ];
          [ zeros(2,2) [1;-1] zeros(2,1) ]];
hx_r5 = [ con.vL_min - con.dLmin*con.dt;
          con.h_reaction;
          -con.h_reaction];
r5 = Polyhedron('A',Hx_r5,'b',hx_r5);

%Create State Dependent Disturbance
Ew_r5 = Ew_r1;
XW_V_r5 = XW_V_r1;

%Region 5.5
Hx_r5_5 = [ 0, 0, 1, 0 ];
hx_r5_5 = [-con.h_reaction];

r5_5 = Polyhedron('A',Hx_r5_5,'b',hx_r5_5);

%=============================

%Create PwDyn Object
dom = Polyhedron('lb',[con.v_min,con.y_min,-h_lim,con.vL_min],'ub',[con.v_max,con.y_max,h_lim,con.vL_max] );

D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin],...
                'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax]); %Feasible disturbances
XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

Ad = {zeros(n_x),zeros(n_x),zeros(n_x)};
            
pwd_A = PwDyn(dom, { r1.intersect(dom), r1_5.intersect(dom), r2.intersect(dom), r3.intersect(dom), r4.intersect(dom), r5.intersect(dom) , r5_5.intersect(dom) } , ...
                { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D , [] , {} , Ew_r1 , XW_V_r1), ...
                  Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D , [] , {} , Ew_r1 , XW_V_r1), ...
                  Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D , [] , {} , Ew_r2 , XW_V_r2), ...
                  Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D , [] , {} , Ew_r3 , XW_V_r3), ...
                  Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D ), ...
                  Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D , [] , {} , Ew_r5 , XW_V_r5), ...
                  Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D , [] , {} , Ew_r5 , XW_V_r5)} );

             
clear A_r1 A_r2 A_r3 A_r4 A_r5 Bw_r1 Bw_r2 Bw_r3 Bw_r4 Bw_r5 F_r1 F_r2 F_r3 F_r4 F_r5

%% Matrix Modifications for the Cautious driver.

%Region 1, for Cautious Driver
A_r1 = [ zeros(3,4) ;
         0 0 0 -1+con.f1*con.dt ];
F_r1 = [zeros(3,1); con.vL_max];

Bw_r1 = { Bw(:,1),Bw(:,2), zeros(n_x,1) };

%Define Polyhedral domain as Hx * x <= h_x
Hx_r1 = [   -(con.K_cau*con.dt + [ 0 0 0 1 ]) ;
            [ zeros(2,3) [1;-1] ] ];
hx_r1 = [   -(con.vL_max - con.dLmax*con.dt);
            con.h_max;
            con.h_max];
r1 = Polyhedron('A',Hx_r1,'b',hx_r1);

%Region 2, for Cautious Driver
A_r2 = [zeros(3,4); con.K_cau*con.dt ];
%%% unsure what is k_C
F_r2 = [zeros(3,1);-con.K3_cau*con.vL_des*con.dt];
Bw_r2 = { Bw(:,1), Bw(:,2), [zeros(3,1); con.dt] };

Hx_r2 = [   con.K_cau*con.dt + [ 0 0 0 1 ] ;
            -(con.K_cau*con.dt + [ 0 0 0 1 ]) ;
            [ zeros(2,3) [1;-1] ] ];
hx_r2 = [   con.vL_max - con.dLmax*con.dt ;
            -con.vL_min + con.dLmin*con.dt;
            con.h_max;
            con.h_max];
r2 = Polyhedron('A',Hx_r2,'b',hx_r2);

%Region 3, for Cautious Driver
A_r3 = [ zeros(3,4) ;
         0 0 0 -1+con.f1*con.dt ];
F_r3 = [zeros(3,1); con.vL_min];

Bw_r3 = {Bw(:,1), Bw(:,2), zeros(n_x,1)};

Hx_r3 = [   con.K_cau*con.dt + [ 0 0 0 1 ];
            [ zeros(2,3) [1;-1] ] ];
hx_r3 = [   con.vL_min - con.dLmin*con.dt;
            con.h_max;
            con.h_max];
r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

%Region 4, for Cautious Driver (Outside of Reaction Zone 1)
A_r4 = [ zeros(3,4) ; con.K_des*con.dt ];
F_r4 = [ zeros(3,1) ; -con.K_des*[0; 0;0;con.vL_des] ];
Bw_r4 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

Hx_r4 = [ zeros(1,3) -1];
hx_r4 = -con.h_max;
r4 = Polyhedron('A',Hx_r4,'b',hx_r4);            
%Region 5, for Cautious Driver (Outside of Reaction Zone 2)
A_r5 = A_r4;
F_r5 = F_r4;
Bw_r5 = Bw_r4;

Hx_r5 = [ zeros(1,3) 1];
hx_r5 = -con.h_max;
r5 = Polyhedron('A',Hx_r5,'b',hx_r5);

%Create PwDyn Object
dom = Polyhedron('lb',[con.v_min, con.y_min, -inf, con.vL_min ], ...
                 'ub',[con.v_max, con.y_max, inf, con.vL_max] );

D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin],'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax]); %Feasible disturbances
XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

pwd_C = PwDyn(dom, { r1 , r2 , r3 , r4 , r5 } , ...
                { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D ), ...
                  Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                  Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D ), ...
                  Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D ), ...
                  Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D )} );


end

