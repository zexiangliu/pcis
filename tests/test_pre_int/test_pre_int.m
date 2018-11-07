clc; clear all; close all;
% to run this code, you need to add the path './get_dyn/' and checkout the
% pre_intersect branch under the pcis

addpath('get_dyn')

%% Parameters

[dyn_a , dyn_c] = get_takeover_pwd();
[regions, dyns_id] = compute_regions(dyn_a,dyn_c);


hmin = 4;
y_lane = 3.6;
l_car = 4.8;
h_r = 300;

react_zone = Polyhedron('H', [0 0 1 0 h_r;
                                 0 0 -1 0 h_r]);

seed_set = react_zone.intersect(Polyhedron('H',[0 1 0 0 y_lane*3/2;
                                                0 -1 0 0 -y_lane/2]));

safe1 = react_zone.intersect(Polyhedron('H', [0 0 -1 0 -hmin;
                                            0 1 0 0 y_lane/2;
                                            0 -1 0 0 y_lane/2]));
safe2 = react_zone.intersect(Polyhedron('H', [0 0 1 0 -hmin;
                                            0 1 0 0 y_lane/2;
                                            0 -1 0 0 y_lane/2]));
                                        
% seed_set = Polyhedron('H',[0 1 0 0 y_lane*3/2;
%                            0 -1 0 0 -y_lane/2]);
% 
% safe1 = Polyhedron('H', [0 0 -1 0 -hmin;
%                          0 1 0 0 y_lane/2;
%                          0 -1 0 0 y_lane/2]);
% safe2 = Polyhedron('H', [0 0 1 0 -hmin;
%                          0 1 0 0 y_lane/2;
%                          0 -1 0 0 y_lane/2]);
                                        
Safe = PolyUnion([seed_set,safe1,safe2]);

rho = 0;

V = seed_set;

%% Set up Inside-out algorithm

max_iter = 5;
iter_num = 1;
vol = volumePolyUnion(V);
vol_new = vol;

% try
%     parpool('local',4)
% catch
%     delete(gcp('nocreate'))
%     parpool('local',4)
% end
%     
profile on;
while(1)
    pre_V = pre_int(dyn_a, dyn_c, V, rho, regions, dyns_id, false);
    V = IntersectPolyUnion(Safe,pre_V);
    vol_new = volumePolyUnion(V);
    if(vol_new == vol && ~isinf(vol) || iter_num >= max_iter)
        break;
    else
        vol = vol_new;
    end
    iter_num = iter_num + 1;
    disp("iter_num: "+num2str(iter_num)+", cumul volume: "+num2str(vol));
end
profile viewer;





%% Extract the range of valid inputs

V_XU = pre_int_xu(dyn_a, dyn_c, V, rho, [], [], true);
% randomly generate a state in V
v = V.Set(1).V;

alpha = rand(1,size(v,1));
alpha = alpha/sum(alpha);

u_range = V_XU.slice([1,2,3,4],(alpha*v(:,1:4))');
plot(u_range)