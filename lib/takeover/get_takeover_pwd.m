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

  %Sanity Check
  % con.aL_max = 100;
  % con.aL_min = -100;

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

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Matrix Modifications for the Annoying driver. %%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %=============================
  %Region 1, for Annoying Driver
  %Region in which the feedback can saturate the upper VELOCITY bound, so
  %its velocity can change arbitrarily in the feasible domain.
  %Region in "front of the car". Constraint is implicit.

  A_r1 = zeros(n_x);
  F_r1 = zeros(n_x,1);

  Bw_r1 = { Bw(:,1), Bw(:,2), zeros(n_x,1) , zeros(n_x,1) };

  %Define Polyhedral domain as Hx * x <= h_x
  Hx_r1 = [ -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ]; 
  hx_r1 = [-(con.vL_max - con.dLmax*con.dt);
            con.h_reaction;
            con.h_reaction];
  r1 = Polyhedron('A',Hx_r1,'b',hx_r1);

  % %Create State Dependent Disturbance
  Ew_r1 = [zeros(3,1);con.dt];
  XW_V_r1 = {[con.K_ann,con.dLmin],(1./con.dt)*[zeros(1,3),-1,con.vL_max]};

  %==========
  %Region 2
  A_r2 = zeros(n_x);
  A_r2(4,:) = -A(4,:);
  F_r2 = zeros(n_x,1);

  Bw_r2 = { Bw(:,1), Bw(:,2), zeros(n_x,1),[zeros(n_x-1,1);1] };

  %Define Polyhedral Domain
  Hx_r2 = [ 0, 0, -1, 0 ];
  hx_r2 = [-con.h_reaction];

  r2 = Polyhedron('A',Hx_r2,'b',hx_r2);    

  %=============================
  %Region 3, for Annoying Driver
  %Region in which:
  % - the feedback is not saturating the VELOCITY bound.
  % - the feedback can saturate the lower bound on ACCELERATION bound.

  A_r3 = zeros(n_x);
  % A_r2(4,:) = con.K_ann*con.dt;

  F_r3 = zeros(n_x,1);
  Bw_r3 = { Bw(:,1), Bw(:,2), zeros(n_x,1),zeros(n_x,1) };

  Hx_r3 = [   con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
              -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
              [ zeros(2,2) [1;-1] zeros(2,1) ] ;
              con.K_ann ;
              con.K_ann ];
  hx_r3 = [   con.vL_max - con.dLmax*con.dt ;
              -con.vL_min + con.dLmin*con.dt;
              con.h_reaction ;
              con.h_reaction ;
              con.aL_min - con.dLmin;
              con.aL_max - con.dLmax];
  r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

  %Create State Dependent Disturbance
  Ew_r3 = [zeros(3,1);con.dt];
  XW_V_r3 = { [zeros(1,4),con.aL_min],[con.K_ann,con.dLmax] };

  %=============================
  %Region 4, for Annoying Driver
  %Region in which:
  % - the feedback is not saturating the VELOCITY bound.
  % - the feedback can saturate the upper bound on ACCELERATION bound.

  A_r4 = zeros(4);
  % A_r3(4,:) = con.K_ann*con.dt;

  F_r4 = zeros(n_x,1);
  Bw_r4 = {Bw(:,1), Bw(:,2), zeros(n_x,1),zeros(n_x,1)};

  Hx_r4 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
            -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ;
            -con.K_ann ;
            -con.K_ann ];
  hx_r4 = [ con.vL_max - con.dLmax*con.dt ;
            -con.vL_min + con.dLmin*con.dt;
            con.h_reaction ;
            con.h_reaction ;
            -(con.aL_min - con.dLmin);
            -(con.aL_max - con.dLmax)];
  r4 = Polyhedron('A',Hx_r4,'b',hx_r4);

  %Create State Dependent
  Ew_r4 = Ew_r3;
  XW_V_r4 = { [con.K_ann,con.dLmin],[zeros(1,4),con.aL_max] };

  %==================================================================================
  %Region 5, for Annoying Driver (Inside of Reaction Zone, All Constraints Satisfied)
  %Region in which:
  % - the feedback is not saturating any VELOCITY bound.
  % - the feedback is not saturating any ACCELERATION bound.

  A_r5 = zeros(n_x);
  A_r5(4,:) = con.K_ann*con.dt;

  F_r5 = zeros(4,1);  
  Bw_r5 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt] , zeros(n_x,1)};

  Hx_r5 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
            -(con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ;
            -con.K_ann ;
            con.K_ann];
  hx_r5 = [ con.vL_max - con.dLmax*con.dt ;
            -con.vL_min + con.dLmin*con.dt;
            con.h_reaction ;
            con.h_reaction ;
            -(con.aL_min - con.dLmin);
            con.aL_max - con.dLmax];
  r5 = Polyhedron('A',Hx_r5,'b',hx_r5);
          
  %==========================================================================================
  %Region 6, for Annoying Driver (Inside of Reaction Zone, Violation of Lower Velocity Bound)
  %Region in which:
  % - the feedback can saturate the lower VELOCITY bound
  %, so its velocity can change arbitrarily in the feasible domain.
  %Region in "front of the car". Constraint is implicit.

  A_r6 = zeros(n_x);
  A_r6(4,:) = -A(4,:);

  F_r6 = F_r1;
  Bw_r6 = Bw_r1;

  Hx_r6 = [ con.K_ann*con.dt + [ 0 0 0 (1-con.f2*con.dt) ];
            [ zeros(2,2) [1;-1] zeros(2,1) ]];
  hx_r6 = [ con.vL_min - con.dLmin*con.dt;
            con.h_reaction;
            con.h_reaction];
  r6 = Polyhedron('A',Hx_r6,'b',hx_r6);

  %Create State Dependent Disturbance
  Ew_r6 = Ew_r1;
  XW_V_r6 = {(1./con.dt)*[zeros(1,3),-1,con.vL_min],[con.K_ann,con.dLmax]};

  %====================================================================
  %Region 7, for Annoying Driver (Outside of Reaction Zone, Behind Car)
  A_r7 = A_r2;
  F_r7 = zeros(n_x,1);

  Bw_r7 = Bw_r2;

  Hx_r7 = [ 0, 0, 1, 0 ];
  hx_r7 = [-con.h_reaction];

  r7 = Polyhedron('A',Hx_r7,'b',hx_r7);

  %=============================

  %Create PwDyn Object
  dom = Polyhedron('lb',[con.v_min,con.y_min,-h_lim,con.vL_min],'ub',[con.v_max,con.y_max,h_lim,con.vL_max] );

  D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin,con.vL_min],...
                  'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax,con.vL_max]); %Feasible disturbances
  XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                  'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

  Ad = {zeros(n_x),zeros(n_x),zeros(n_x),zeros(n_x)};
              
  pwd_A = PwDyn(dom, { r1.intersect(dom), r2.intersect(dom), r3.intersect(dom), r4.intersect(dom), r5.intersect(dom), r6.intersect(dom) , r7.intersect(dom) } , ...
                  { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                    Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                    Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D , [] , {} , Ew_r3 , XW_V_r3), ...
                    Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D , [] , {} , Ew_r4 , XW_V_r4), ...
                    Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D ), ...
                    Dyn(A+A_r6, F_r6, B, XU , {} , {} , Polyhedron(), Ad, Bw_r7 , D ), ...
                    Dyn(A+A_r7, F_r7, B, XU , {} , {} , Polyhedron(), Ad, Bw_r7 , D )} );

  %Clearing all variables that are overwritten for next intention          
  clear A_r1 A_r2 A_r3 A_r4 A_r5 Bw_r1 Bw_r2 Bw_r3 Bw_r4 Bw_r5 F_r1 F_r2 F_r3 F_r4 F_r5

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Matrix Modifications for the Cautious driver.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %=============================
  %Region 1, for Cautious Driver
  %Region in which:
  % - the feedback can saturate the upper VELOCITY bound, so
  %its velocity can change arbitrarily in the feasible domain.

  A_r1 = zeros(n_x);
  F_r1 = zeros(n_x,1);

  Bw_r1 = { Bw(:,1), Bw(:,2), zeros(n_x,1) , zeros(n_x,1) };

  %Define Polyhedral domain as Hx * x <= h_x
  Hx_r1 = [ -(con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ]; 
  hx_r1 = [-(con.vL_max + (con.K_des*[0;0;0;con.vL_des]-con.dLmax)*con.dt);
            con.h_reaction;
            con.h_reaction];
  r1 = Polyhedron('A',Hx_r1,'b',hx_r1);

  %Create State Dependent Disturbance
  %In region 1, the input is an acceleration, so the gain term "Ew_r1" should contain a con.dt in it
  Ew_r1 = [zeros(3,1);con.dt];
  XW_V_r1 = {[con.K_cau,con.dLmin-con.K_des*[zeros(3,1);con.vL_des]],(1./con.dt)*[zeros(1,3),-1,con.vL_max]};

  %=============================
  %Region 2, for Cautious Driver
  %Region in which:
  % - the lead car is far behind the lead car
  % - the ego car is outside of the lead car's reaction zone.

  A_r2 = zeros(n_x);
  A_r2(4,:) = -A(4,:); %The velocity will be directly controlled by the state-independent disturbance.
  F_r2 = zeros(n_x,1);

  Bw_r2 = { Bw(:,1), Bw(:,2), zeros(n_x,1),[zeros(n_x-1,1);1] };

  %Define Polyhedral Domain
  Hx_r2 = [ 0, 0, -1, 0 ];
  hx_r2 = [-con.h_reaction];

  r2 = Polyhedron('A',Hx_r2,'b',hx_r2);    

  %=============================
  %Region 3, for Cautious Driver
  %Region in which:
  % - the feedback is not saturating the VELOCITY bound.
  % - the feedback can saturate the lower bound on ACCELERATION.
  % - the ego car IS IN the lead car's reaction zone.

  A_r3 = zeros(n_x);
  % A_r2(4,:) = con.K_cau*con.dt;

  F_r3 = zeros(n_x,1);
  Bw_r3 = { Bw(:,1), Bw(:,2), zeros(n_x,1),zeros(n_x,1) };

  Hx_r3 = [   con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
              -(con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
              [ zeros(2,2) [1;-1] zeros(2,1) ] ;
              con.K_cau ;
              con.K_cau ];
  hx_r3 = [   con.vL_max + (con.K_des*[0;0;0;con.vL_des]- con.dLmax)*con.dt ;
              -(con.vL_min - (con.K_des*[0;0;0;con.vL_des] - con.dLmin)*con.dt);
              con.h_reaction ;
              con.h_reaction ;
              con.aL_min - con.dLmin + con.K_des*[0;0;0;con.vL_des];
              con.aL_max - con.dLmax + con.K_des*[0;0;0;con.vL_des]];
  r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

  %Create State Dependent Disturbance
  %In region 3, the input is an acceleration, so the gain term "Ew_r3" should contain a con.dt in it.
  Ew_r3 = [zeros(3,1);con.dt];
  XW_V_r3 = { [zeros(1,4),con.aL_min],[con.K_cau,con.dLmax-con.K_des*[0;0;0;con.vL_des]] };

  %=============================
  %Region 4, for Cautious Driver
  %Region in which:
  % - the feedback is not saturating the VELOCITY bound.
  % - the feedback can saturate the upper bound on ACCELERATION bound.

  A_r4 = zeros(4);
  % A_r3(4,:) = con.K_cau*con.dt;

  F_r4 = zeros(n_x,1);
  Bw_r4 = {Bw(:,1), Bw(:,2), zeros(n_x,1),zeros(n_x,1)};

  Hx_r4 = [ con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
            -(con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ;
            -con.K_cau ;
            -con.K_cau ];
  hx_r4 = [ (con.vL_max+ (con.K_des*[0;0;0;con.vL_des] - con.dLmax)*con.dt) ;
            (-con.vL_min+ (-con.K_des*[0;0;0;con.vL_des] +con.dLmin)*con.dt);
            con.h_reaction ;
            con.h_reaction ;
            -(con.aL_min - con.dLmin+con.K_des*[0;0;0;con.vL_des]) ;
            -(con.aL_max - con.dLmax+con.K_des*[0;0;0;con.vL_des])];
  r4 = Polyhedron('A',Hx_r4,'b',hx_r4);

  %Create State Dependent
  Ew_r4 = Ew_r3;
  XW_V_r4 = { [con.K_cau,con.dLmin-con.K_des*[0;0;0;con.vL_des]],[zeros(1,4),con.aL_max] };

  %==================================================================================
  %Region 5, for Cautious Driver (Inside of Reaction Zone, All Constraints Satisfied)
  %Region in which:
  % - the feedback is not saturating any VELOCITY bound.
  % - the feedback is not saturating any ACCELERATION bound.

  A_r5 = zeros(n_x);
  A_r5(4,:) = con.K_cau*con.dt;

  F_r5 = [ zeros(3,1) ; -con.K_des*[0; 0;0;con.vL_des]*con.dt ]; 
  Bw_r5 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt] , zeros(n_x,1)};

  Hx_r5 = [ con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ] ;
            -(con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ]) ;
            [ zeros(2,2) [1;-1] zeros(2,1) ] ;
            -con.K_cau ;
            con.K_cau];
  hx_r5 = [ con.vL_max+ (con.K_des*[0;0;0;con.vL_des] - con.dLmax)*con.dt ;
            -con.vL_min+ (-con.K_des*[0;0;0;con.vL_des]+con.dLmin)*con.dt;
            con.h_reaction ;
            con.h_reaction ;
            -(con.aL_min - con.dLmin+con.K_des*[0;0;0;con.vL_des]);
            con.aL_max - con.dLmax+con.K_des*[0;0;0;con.vL_des]];
  r5 = Polyhedron('A',Hx_r5,'b',hx_r5);
          
  %==========================================================================================
  %Region 6, for Cautious Driver (Inside of Reaction Zone, Violation of Lower Velocity Bound)
  %Region in which:
  % - the feedback can saturate the lower VELOCITY bound
  %, so its velocity can change arbitrarily in the feasible domain.
  %Region in "front of the car". Constraint is implicit.

  A_r6 = zeros(n_x);
  A_r6(4,:) = -A(4,:);

  F_r6 = F_r1;
  Bw_r6 = Bw_r1;

  Hx_r6 = [ con.K_cau*con.dt + [ 0 0 0 (1-con.f2*con.dt) ];
            [ zeros(2,2) [1;-1] zeros(2,1) ]];
  hx_r6 = [ con.vL_min+ (con.K_des*[0;0;0;con.vL_des]- con.dLmin)*con.dt;
            con.h_reaction;
            con.h_reaction];
  r6 = Polyhedron('A',Hx_r6,'b',hx_r6);

  %Create State Dependent Disturbance
  Ew_r6 = Ew_r1;
  XW_V_r6 = {(1./con.dt)*[zeros(1,3),-1,con.vL_min],[con.K_cau,con.dLmax-con.K_des*[0;0;0;con.vL_des]]};

  %====================================================================
  %Region 7, for Cautious Driver (Outside of Reaction Zone, Behind Car)
  A_r7 = A_r2;
  F_r7 = zeros(n_x,1);

  Bw_r7 = Bw_r2;

  Hx_r7 = [ 0, 0, 1, 0 ];
  hx_r7 = [-con.h_reaction];

  r7 = Polyhedron('A',Hx_r7,'b',hx_r7);

  %=============================

  %Create PwDyn Object
  dom = Polyhedron('lb',[con.v_min,con.y_min,-h_lim,con.vL_min],'ub',[con.v_max,con.y_max,h_lim,con.vL_max] );

  D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin,con.vL_min],...
                  'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax,con.vL_max]); %Feasible disturbances
  XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                  'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

  Ad = {zeros(n_x),zeros(n_x),zeros(n_x),zeros(n_x)};
              
  pwd_C = PwDyn(dom, { r1.intersect(dom), r2.intersect(dom), r3.intersect(dom), r4.intersect(dom), r5.intersect(dom), r6.intersect(dom) , r7.intersect(dom) } , ...
                  { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D , [] , {} , Ew_r1 , XW_V_r1), ...
                    Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                    Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D , [] , {} , Ew_r3 , XW_V_r3), ...
                    Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D , [] , {} , Ew_r4 , XW_V_r4), ...
                    Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D ), ...
                    Dyn(A+A_r6, F_r6, B, XU , {} , {} , Polyhedron(), Ad, Bw_r6 , D , [] , {} , Ew_r6 , XW_V_r6), ...
                    Dyn(A+A_r7, F_r7, B, XU , {} , {} , Polyhedron(), Ad, Bw_r7 , D )} );

end

