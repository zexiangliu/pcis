function plot_CIS(CIS_a, CIS_ac)
figure;
hold on;
vEgo = 16;
vLead = 25;

plot(Polyhedron('ub', [0.9 4], 'lb', [-0.9 -4]), 'Color','r', 'alpha', 1)

for c = 1:CIS_a.Num
    cc = intersect(CIS_a.Set(c).slice([1 4], [vEgo vLead]), Polyhedron('ub', [2.7 50], 'lb', [-1 -50]));
    plot(cc, 'Color', 'b', 'alpha', 1, 'edgealpha',0, 'marker','none');
end

for c = 1:CIS_ac.Num
    cc = intersect(CIS_ac.Set(c).slice([1 4], [vEgo vLead]), Polyhedron('ub', [2.7 50], 'lb', [-1 -50]));
    plot(cc, 'Color', 'c', 'alpha', 1, 'edgealpha',0, 'marker','none');
end
% plot(CIS_a.slice([1 4], [vEgo vLead]), 'Color', 'b', 'alpha', .2, 'edgealpha',0, 'marker','none');
% plot(CIS_ac.slice([1 4], [25 25]), 'Color', 'r', 'alpha', 1, 'edgealpha',0, 'marker','none');

plot([-0.9 -0.9], [-50 50], 'k-', 'LineWidth', 5);
plot([0.9 0.9], [-50 50], 'k--', 'LineWidth', 5);
plot(0.5*[5.4 5.4], [-50 50], 'k-', 'LineWidth', 5);
set(gca,'Xdir','reverse','Ydir','reverse')
axis([-1 2.8 -50 50]);
xlabel('ye'); ylabel('h');
title(['vEgo =  ', num2str(vEgo), ' m/s, vLead = ', num2str(vLead), ' m/s']);
