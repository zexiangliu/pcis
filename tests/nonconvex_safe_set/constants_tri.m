function [con] = constants_tri()
  % Constants taken from the current TRI ACC model

  % sampling time
  con.dt = 0.1;

  % car dynamics 
  % longitudinal
  con.f1 = 0.1;
  con.K_ann = 0.0012;
  con.K_cau = [0.01 1.5 0.0008];
  con.K1_cau = con.K_cau(1);
  con.K2_cau = con.K_cau(2);
  con.K3_cau = con.K_cau(3);
  con.vL_des = 25;
  % lateral
  con.y_min = -0.9;
  con.y_max = 4.5;
  con.y_lane = 3.6; % lane width
  
  % assumptions
  con.vL_min = 0;   % Lead car min velocity (m/s)
  con.vL_max = 25;  % Lead car max velocity (m/s)
  con.aL_min = -3; % m/s^2
  con.aL_max = 3;  % m/s^2
  
  % state/input/disturbance bounds
  con.v_min = 16;
  con.v_max = 30;
  con.umin_ACC = -3;
  con.umax_ACC = 3;
  con.umax_LK = 0.9; 
  con.umin_LK = -0.9;
  con.dmax_ACC = 0.05*con.umax_ACC;
  con.dmin_ACC = 0.05*con.umin_ACC;
  con.dmax_LK = 0.05*con.umax_LK;
  con.dmin_LK = 0.05*con.umin_LK;
  con.dLmax = 0.05*con.aL_max;
  con.dLmin = 0.05*con.aL_min;
    
  % specifications
  con.h_min = 4;   % minimum distance behind/ahead of lead car
  con.h_max = 300; % max distance behind/ahead of lead car (m)
  con.tau_min = 1.7; % minimum
  con.tau_des = 2.1;
  
end