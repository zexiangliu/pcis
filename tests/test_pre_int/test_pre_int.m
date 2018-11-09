clc; clear all; close all;
% to run this code, you need to add the path './get_dyn/' and checkout the
% pre_intersect branch under the pcis

addpath('get_dyn')
con = constants_tri();

%% Parameters

[dyn_a , dyn_c] = get_takeover_pwd();
[regions, dyns_id] = compute_regions(dyn_c,dyn_a);

% for i = 1:length(dyns_id)
%     dyns_id{i} = [dyns_id{i}(2),dyns_id{i}(2)];
% end

% 
% hmin = 4;
% y_lane = 3.6;
% l_car = 4.8;
h_r = 300;
% 
react_zone = Polyhedron('H', [0 0 1 0 h_r;
                                 0 0 -1 0 h_r]);
% % 
% % seed_set = react_zone.intersect(Polyhedron('H',[0 1 0 0 y_lane*3/2;
% %                                                 0 -1 0 0 -y_lane/2]));
% % 
% % safe1 = react_zone.intersect(Polyhedron('H', [0 0 -1 0 -hmin;
% %                                             0 1 0 0 y_lane/2;
% %                                             0 -1 0 0 y_lane/2]));
% % safe2 = react_zone.intersect(Polyhedron('H', [0 0 1 0 -hmin;
% %                                             0 1 0 0 y_lane/2;
% %                                             0 -1 0 0 y_lane/2]));
%                                         
% seed_set = Polyhedron('H',[0 1 0 0 y_lane*3/2;
%                            0 -1 0 0 -y_lane/2]);
% 
% safe1 = Polyhedron('H', [0 0 -1 0 -hmin;
%                          0 1 0 0 y_lane*3/2;
%                          0 -1 0 0 y_lane/2]);
% safe2 = Polyhedron('H', [0 0 1 0 -hmin;
%                          0 1 0 0 y_lane*3/2;
%                          0 -1 0 0 y_lane/2]);
%                                         
% Safe = PolyUnion([seed_set,safe1,safe2]);
% 
% 
% 
% rho = 0;
% 
% V = seed_set;



%% Create Safe Set and Small Invariant Set
% 
% X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
%                 'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
% X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
%                 'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
% X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
%                 'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);
% 
% % Safe set 
% Safe = PolyUnion([X1 X2 X3]);
% 
% % cinv set
% V = X2;
% rho = 0;


%% Create Safe Set and Small Invariant Set

X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
                'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
                'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);

% Safe set 
Safe = PolyUnion([X1 X2 X3]);

% cinv set
V = PolyUnion(X2);
rho = 0;

%% Set up Inside-out algorithm

max_iter = 4;
iter_num = 1;
vol = volumePolyUnion(V);
% vol_new = vol;

% try
%     parpool('local',4)
% catch
%     delete(gcp('nocreate'))
%     parpool('local',4)
% end

fig =figure;

profile on;
counter = 0;
while(1)
    counter = counter + 1;
    pre_V = pre_int(dyn_c, dyn_a, V, rho, regions, dyns_id, true);
    V_old = V;
%     V = IntersectPolyUnion(Safe,pre_V);
    tmp_V = IntersectPolyUnion(Safe,pre_V);
    V = PolyUnion([V.Set,tmp_V.Set]);
    V_saved = V;
%     try
% %         V.reduce();
% %         V.merge();
%     catch
%         V = V_saved;
%         V.reduce();
%     end
    if(mod(counter,5)==0)
      V_old = IntersectPolyUnion(V_old, react_zone);
      vol = volumePolyUnion(setMinus3(...
          IntersectPolyUnion(V,react_zone),V_old));
    end
    
%     fig = figure;
    visual(V,fig);
%     ylim([-30 30])
    if(vol == 0 || iter_num >= max_iter)
        break;
    end
    iter_num = iter_num + 1;
    disp("iter_num: "+num2str(iter_num)+", residual volume: "+num2str(vol));
end
profile viewer;


