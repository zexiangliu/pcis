clc; clear all; close all;
% to run this code, you need to add the path './get_dyn/' and checkout the
% pre_intersect branch under the pcis

addpath('get_dyn')
con = constants_tri();

%{
  6×1 cell array

    {'LCP'    }
    {'CDD'    }
    {'MOSEK'  }
    {'GLPK'   }
    {'LINPROG'}
    {'SEDUMI' }
%}

mptopt('lpsolver', 'LCP', 'qpsolver', 'LCP');

%% Parameters

[dyn_a , dyn_c] = get_takeover_pwd();
[regions, dyns_id] = compute_regions(dyn_c,dyn_a);

% for i = 1:length(dyns_id)
%     dyns_id{i} = [dyns_id{i}(2),dyns_id{i}(2)];
% end

react_zone = Polyhedron('H', [0 0 1 0 con.h_max;
                                 0 0 -1 0 con.h_max]);

%% Create Safe Set and Small Invariant Set

% X1 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;      Inf],...
%                 'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
% X2 = Polyhedron('UB', [con.v_max;   con.y_max;      con.h_max;     Inf],...
%                 'LB', [con.v_min;   -con.y_min;     -con.h_max;    -Inf]);
% X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
%                 'LB', [con.v_min;   con.y_min;      -con.h_max;    -Inf]);

X1 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf;      Inf],...
                'LB', [con.v_min;   con.y_min;      con.h_min;     -Inf]);
X2 = Polyhedron('UB', [con.v_max;   con.y_max;      Inf;     Inf],...
                'LB', [con.v_min;   -con.y_min;     -Inf;    -Inf]);
X3 = Polyhedron('UB', [con.v_max;   con.y_max;      -con.h_min;    Inf],...
                'LB', [con.v_min;   con.y_min;      -Inf;    -Inf]);
% Safe set 
Safe = PolyUnion([X1 X3]);

% cinv set
V = PolyUnion(X2);
rho = 1e-6;

%% Set up Inside-out algorithm

max_iter = 6;
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

collector = [];

V_all = PolyUnion(V.Set);

% profile on;
counter = 0;
while(1)
    counter = counter + 1;
    
    [pre_V] = pre_int(dyn_c, dyn_a, V, rho, regions, dyns_id, false);
    
    V_old = V;
    V = IntersectPolyUnion(X1,pre_V);
%     V = V_all;
    
    V_saved = V
    
    visual(V,fig);
%     V = PolyUnion([collector, V.Set]);
    try
        V.merge()
%         V.reduce();
    catch
        V = V_saved;
%         V.reduce();
    end
    
    V_all = PolyUnion([V_all.Set,V.Set]);
%     new_V = [];
%     for i = 1:V.Num
%     if V.Set(i).volume<=2000
%         collector = [collector V.Set(i)];
%      else
%         new_V = [new_V V.Set(i)];
%      end
%     end    
%     V = PolyUnion(new_V)
    
    
%     if(mod(counter,5)==0)
%       V_old = IntersectPolyUnion(V_old, react_zone);
%       vol = volumePolyUnion(setMinus3(...
%           IntersectPolyUnion(V,react_zone),V_old));
%     end
%     
%     fig = figure;
%     ylim([-30 30])
    if(vol == 0 || iter_num >= max_iter)
        break;
    end
    iter_num = iter_num + 1;
    disp("iter_num: "+num2str(iter_num)+", residual volume: "+num2str(vol));
end
% profile viewer;


