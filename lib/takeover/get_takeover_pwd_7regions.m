function [ pwd_A , pwd_C ] = get_takeover_pwd_7regions( varargin )
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
  
  % vla: vl saturated from above
  % vl: vl is not saturated
  % vlb: vl saturated from below

  % ala: al saturated from above
  % al: al not saturated
  % alb: al saturated from below
  
  %=============================
  % Region 1 (vl, al) - Annoying Driver
  
  A_r1 = [zeros(n_x-1, n_x);con.K_ann*con.dt]; % add state feedback
  Bw_r1 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt]}; % dist
  F_r1 = zeros(4,1);  % no affine term


  Hx_r1 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            con.K_ann*con.dt + A(4,:); % vl + al < vLmax
            -(con.K_ann*con.dt + A(4,:)); % vl + al > vLmin
            con.K_ann; % al < almax
            -con.K_ann]; % al > almin
  hx_r1 = [  con.h_reaction ;
             con.h_reaction ;
             con.vL_max - con.dLmax*con.dt ;
            -(con.vL_min - con.dLmin*con.dt);
             con.aL_max - con.dLmax; 
           -(con.aL_min - con.dLmin)];
  r1 = Polyhedron('A',Hx_r1,'b',hx_r1);
  
  
  %=============================
  % Region 2 (vla, ala) - Annoying Driver
  % do not use state feedback
  % set vL = vL_max - dLmax*dt

  A_r2 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r2 = [zeros(n_x-1,1); con.vL_max - con.dLmax*con.dt]; % set vL = vL_max - dLmax*dt
  Bw_r2 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r2 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            -A(4,:); % vl + almax > vLmax
            -con.K_ann]; % al > almax
  hx_r2 = [  con.h_reaction ;
             con.h_reaction ;
            -(con.vL_max - con.dLmax*con.dt - con.aL_max*con.dt);
            -(con.aL_max - con.dLmax)];
       
  r2 = Polyhedron('A',Hx_r2,'b',hx_r2);

  %==========
  % Region 3 (vla, al) - Annoying Driver
  % do not use state feedback
  % set vL = vL_max - dLmax*dt + dL

  A_r3 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r3 = [zeros(n_x-1,1); con.vL_max - con.dLmax*con.dt]; % set vL = vL_max - dLmax*dt + dL
  Bw_r3 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r3 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            -(con.K_ann*con.dt + A(4,:)); % vl + al > vLmax
            con.K_ann; % al < almax
            -con.K_ann]; % al > almin
  hx_r3 = [  con.h_reaction ;
             con.h_reaction ;
            -(con.vL_max - con.dLmax*con.dt);
            (con.aL_max - con.dLmax);
            -(con.aL_min - con.dLmin)]; 

  r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

  %=============================
  % Region 4 (vl - ala)
  % do not use state feedback
  % set al = al_max
  
  A_r4 = zeros(n_x); % do not use state feedback
  F_r4 = [zeros(n_x-1, 1); con.aL_max*con.dt]; % but use a fixed al = almax
  Bw_r4 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt] };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r4 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almax < vLmax
            -(A(4,:)); % vl + almax > vLmin
            -con.K_ann]; % al > al_max
  hx_r4 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_max - con.dLmax*con.dt - con.aL_max*con.dt);
            -(con.vL_min - con.dLmin*con.dt - con.aL_max*con.dt);
            -(con.aL_max - con.dLmax)]; 

  r4 = Polyhedron('A',Hx_r4,'b',hx_r4);

  %=============================
  % Region 5 (vl, alb)
  % do not use state feedback
  % set al = al_min
    
  A_r5 = zeros(n_x); % do not use state feedback
  F_r5 = [zeros(n_x-1, 1); con.aL_min*con.dt]; % but use a fixed al = almin
  Bw_r5 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt] };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r5 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almin < vLmax
            -(A(4,:)); % vl + almin > vLmin
            con.K_ann]; % al < al_min
  hx_r5 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_max - con.dLmax*con.dt - con.aL_min*con.dt);
            -(con.vL_min - con.dLmin*con.dt - con.aL_min*con.dt);
            (con.aL_min - con.dLmin)]; 

  r5 = Polyhedron('A',Hx_r5,'b',hx_r5);

  %=============================
  % Region 6 (vlb, al) - Annoying Driver
  % do not use state feedback
  % set vL = vL_min - dLmin*dt

  A_r6 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r6 = [zeros(n_x-1,1); con.vL_min - con.dLmin*con.dt]; % set vL = vL_min - dLmin*dt
  Bw_r6 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r6 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (con.K_ann*con.dt + A(4,:)); % vl + al < vLmin
             con.K_ann; % al < almax
            -con.K_ann]; % al > almin
  hx_r6 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_min - con.dLmin*con.dt);
            (con.aL_max - con.dLmax);
            -(con.aL_min - con.dLmin)];
       
  r6 = Polyhedron('A',Hx_r6,'b',hx_r6);

  %==========
  % Region 7 (vlb, alb) - Annoying Driver
  % do not use state feedback
  % set vL = vL_min - dLmin*dt + dL

  A_r7 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
%   F_r7 = [zeros(n_x-1,1); con.vL_max - con.dLmax*con.dt]; % set vL = vL_max - dLmax*dt + dL
  F_r7 = [zeros(n_x-1,1); con.vL_min - con.dLmin*con.dt]; % set vL = vL_min - dLmin*dt

  Bw_r7 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]};

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r7 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almin < vLmin
            con.K_ann]; % al < almin
  hx_r7 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_min - con.dLmin*con.dt - con.aL_min*con.dt);
            (con.aL_min - con.dLmin)]; 

  r7 = Polyhedron('A',Hx_r7,'b',hx_r7);

  %=============================

  %Create PwDyn Object
  dom = Polyhedron('lb',[con.v_min,con.y_min,-h_lim,con.vL_min],'ub',[con.v_max,con.y_max,h_lim,con.vL_max] );

  D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin],...
                  'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax]); %Feasible disturbances
  XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                  'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

  Ad = {zeros(n_x),zeros(n_x),zeros(n_x)};
              
  pwd_A = PwDyn(dom, { r1.intersect(dom), r2.intersect(dom), r3.intersect(dom), r4.intersect(dom), r5.intersect(dom), r6.intersect(dom) , r7.intersect(dom) } , ...
                  { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D ), ...
                    Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                    Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D ), ...
                    Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D ), ...
                    Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D ), ...
                    Dyn(A+A_r6, F_r6, B, XU , {} , {} , Polyhedron(), Ad, Bw_r6 , D ), ...
                    Dyn(A+A_r7, F_r7, B, XU , {} , {} , Polyhedron(), Ad, Bw_r7 , D )} );

  %Clearing all variables that are overwritten for next intention          
  clear A_r1 A_r2 A_r3 A_r4 A_r5 A_r6 A_r7 Bw_r1 Bw_r2 Bw_r3 Bw_r4 Bw_r5 Bw_r6 Bw_r7 F_r1 F_r2 F_r3 F_r4 F_r5 F_r6 F_r7

  
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %% Matrix Modifications for the Cautious driver. %%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  % vla: vl saturated from above
  % vl: vl is not saturated
  % vlb: vl saturated from below

  % ala: al saturated from above
  % al: al not saturated
  % alb: al saturated from below
  
  % al = Kcau*q - kcau*vL_des
  % vl+ = vl + al*dt
  %=============================
  % Region 1 (vl, al) - Cautious Driver
  
  A_r1 = [zeros(n_x-1, n_x);con.K_cau*con.dt]; % add state feedback
  Bw_r1 = {Bw(:,1), Bw(:,2), [0;0;0;con.dt]}; % dist
  F_r1 = [zeros(3,1); -con.k_cau*con.vL_des*con.dt]; % al = Kcau*q - kcau*vL_des


  Hx_r1 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            con.K_cau*con.dt + A(4,:); % vl + al < vLmax
            -(con.K_cau*con.dt + A(4,:)); % vl + al > vLmin
            con.K_cau; % al < almax
            -con.K_cau]; % al > almin
  hx_r1 = [  con.h_reaction ;
             con.h_reaction ;
             con.vL_max + con.k_cau*con.vL_des*con.dt - con.dLmax*con.dt;
            -(con.vL_min + con.k_cau*con.vL_des*con.dt - con.dLmin*con.dt);
             con.aL_max + con.k_cau*con.vL_des - con.dLmax; 
           -(con.aL_min + con.k_cau*con.vL_des - con.dLmin)];
  r1 = Polyhedron('A',Hx_r1,'b',hx_r1);
  
  
  %=============================
  % Region 2 (vla, ala) - Cautious Driver
  % do not use state feedback
  % set vL = vL_max - dLmax*dt

  A_r2 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r2 = [zeros(n_x-1,1); con.vL_max - con.dLmax*con.dt];% set vL = vL_max - dLmax*dt + dL
  Bw_r2 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r2 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            -A(4,:); % vl + almax > vLmax
            -con.K_cau]; % al > almax
  hx_r2 = [  con.h_reaction ;
             con.h_reaction ;
            -(con.vL_max - con.dLmax*con.dt - con.aL_max*con.dt);
            -(con.aL_max + con.k_cau*con.vL_des - con.dLmax)];
       
  r2 = Polyhedron('A',Hx_r2,'b',hx_r2);

  %==========
  % Region 3 (vla, al) - Cautious Driver
  % do not use state feedback
  % set vL = vL_max - dLmax*dt + dL

  A_r3 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r3 = [zeros(n_x-1,1); con.vL_max - con.dLmax*con.dt]; % set vL = vL_max - dLmax*dt + dL
  Bw_r3 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r3 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            -(con.K_cau*con.dt + A(4,:)); % vl + al > vLmax
            con.K_cau; % al < almax
            -con.K_cau]; % al > almin
  hx_r3 = [  con.h_reaction ;
             con.h_reaction ;
            -(con.vL_max + con.k_cau*con.vL_des*con.dt - con.dLmax*con.dt);
            (con.aL_max + con.k_cau*con.vL_des - con.dLmax);
            -(con.aL_min + con.k_cau*con.vL_des - con.dLmin)]; 

  r3 = Polyhedron('A',Hx_r3,'b',hx_r3);

  %=============================
  % Region 4 (vl - ala) - Cautious Driver
  % do not use state feedback
  % set al = al_max
  
  A_r4 = zeros(n_x); % do not use state feedback
  F_r4 = [zeros(n_x-1, 1); con.aL_max*con.dt]; % but use a fixed al = almax
  Bw_r4 = { Bw(:,1), Bw(:,2),[0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r4 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almax < vLmax
            -(A(4,:)); % vl + almax > vLmin
            -con.K_cau]; % al > al_max
  hx_r4 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_max - con.dLmax*con.dt - con.aL_max*con.dt);
            -(con.vL_min - con.dLmin*con.dt - con.aL_max*con.dt);
            -(con.aL_max + con.k_cau*con.vL_des - con.dLmax)]; 

  r4 = Polyhedron('A',Hx_r4,'b',hx_r4);

  %=============================
  % Region 5 (vl, alb) - Cautious Driver
  % do not use state feedback
  % set al = al_min
    
  A_r5 = zeros(n_x); % do not use state feedback
  F_r5 = [zeros(n_x-1, 1); con.aL_min*con.dt]; % but use a fixed al = almin
  Bw_r5 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r5 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almin < vLmax
            -(A(4,:)); % vl + almin > vLmin
            con.K_cau]; % al < al_min
  hx_r5 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_max - con.dLmax*con.dt - con.aL_min*con.dt);
            -(con.vL_min - con.dLmin*con.dt - con.aL_min*con.dt);
            (con.aL_min + con.k_cau*con.vL_des - con.dLmin)]; 

  r5 = Polyhedron('A',Hx_r5,'b',hx_r5);

  %=============================
  % Region 6 (vlb, al) - Cautious Driver
  % do not use state feedback
  % set vL = vL_min - dLmin*dt

  A_r6 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r6 = [zeros(n_x-1,1); con.vL_min - con.dLmin*con.dt]; % set vL = vL_min - dLmin*dt
  Bw_r6 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r6 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (con.K_cau*con.dt + A(4,:)); % vl + al < vLmin
             con.K_cau; % al < almax
            -con.K_cau]; % al > almin
  hx_r6 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_min + con.k_cau*con.vL_des*con.dt - con.dLmin*con.dt);
            (con.aL_max + con.k_cau*con.vL_des - con.dLmax);
            -(con.aL_min + con.k_cau*con.vL_des - con.dLmin)];
       
  r6 = Polyhedron('A',Hx_r6,'b',hx_r6);

  %==========
  % Region 7 (vlb, alb) - Cautious Driver
  % do not use state feedback
  % set vL = vL_min - dLmin*dt + dL

  A_r7 = [zeros(n_x-1, n_x); -A(4,:)]; % do not use state feedback
  F_r7 = [zeros(n_x-1,1); con.vL_min - con.dLmin*con.dt]; % set vL = vL_min - dLmin*dt

  Bw_r7 = { Bw(:,1), Bw(:,2), [0;0;0;con.dt]  };

  %Define Polyhedral domain as Hx * x <= h_x
 
  Hx_r7 = [ [ zeros(2,2) [1;-1] zeros(2,1) ] ; % -|hr| < |h| < |hr|
            (A(4,:)); % vl + almin < vLmin
            con.K_cau]; % al < almin
  hx_r7 = [  con.h_reaction ;
             con.h_reaction ;
            (con.vL_min - con.dLmin*con.dt - con.aL_min*con.dt);
            (con.aL_min + con.k_cau*con.vL_des - con.dLmin)]; 

  r7 = Polyhedron('A',Hx_r7,'b',hx_r7);

  %=============================

  %Create PwDyn Object
  dom = Polyhedron('lb',[con.v_min,con.y_min,-h_lim,con.vL_min],'ub',[con.v_max,con.y_max,h_lim,con.vL_max] );

  D = Polyhedron('lb',[con.dmin_ACC,con.dmin_LK,con.dLmin],...
                  'ub',[con.dmax_ACC,con.dmax_LK,con.dLmax]); %Feasible disturbances
  XU = Polyhedron('A',[zeros(n_u,n_x) eye(n_u) ; zeros(n_u,n_x) -eye(n_u) ], ...
                  'b',[con.umax_ACC ; con.umax_LK ; -con.umin_ACC ; -con.umin_LK ]);

  Ad = {zeros(n_x),zeros(n_x),zeros(n_x)};
              
  pwd_C = PwDyn(dom, { r1.intersect(dom), r2.intersect(dom), r3.intersect(dom), r4.intersect(dom), r5.intersect(dom), r6.intersect(dom) , r7.intersect(dom) } , ...
                  { Dyn(A+A_r1, F_r1, B, XU , {} , {} , Polyhedron(), Ad, Bw_r1 , D ), ...
                    Dyn(A+A_r2, F_r2, B, XU , {} , {} , Polyhedron(), Ad, Bw_r2 , D ), ...
                    Dyn(A+A_r3, F_r3, B, XU , {} , {} , Polyhedron(), Ad, Bw_r3 , D ), ...
                    Dyn(A+A_r4, F_r4, B, XU , {} , {} , Polyhedron(), Ad, Bw_r4 , D ), ...
                    Dyn(A+A_r5, F_r5, B, XU , {} , {} , Polyhedron(), Ad, Bw_r5 , D ), ...
                    Dyn(A+A_r6, F_r6, B, XU , {} , {} , Polyhedron(), Ad, Bw_r6 , D ), ...
                    Dyn(A+A_r7, F_r7, B, XU , {} , {} , Polyhedron(), Ad, Bw_r7 , D )});
end

