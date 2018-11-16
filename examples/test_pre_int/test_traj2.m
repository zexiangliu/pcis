% Aim to figure out why the sample trajectory is unsafe
clc;clear all;close all;
load CIS_ac.mat
load CIS_collection_XU_v2_8iter.mat
load CIS_ac_XU.mat
load sample_trajectory.mat

mptopt('lpsolver', 'CDD', 'qpsolver', 'LCP');

%% 

valid_x = zeros(length(time)-1,1);
valid_xu = zeros(length(time)-1,1);

for i = 1:length(time)-1
    i
    x = [vEgo(i), yEgo(i), h(i), vLead(i)]';
    u = [aEgo(i), steering(i)]';
    if containsPolyUnion(CIS_ann,x)
        valid_x(i) = 1;        

        if containsPolyUnion(preXU_ann,[x;u])
            valid_xu(i) = 1;
        end
    end
end