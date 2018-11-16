function [x,u] = get_from_traj(idx)

data = load('sample_trajectory.mat');

x = zeros(4,1);

x(1) = data.vEgo(idx);
x(2) = data.yEgo(idx);
x(3) = data.h(idx);
x(4) = data.vLead(idx);

u = zeros(2,1);
u(1) = data.aEgo(idx);
u(2) = data.steering(idx);
end