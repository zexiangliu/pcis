function plotU(x, PU, fig)
% x is a point, PU is polyUnion

if nargin == 2
    figure;hold on;
else
    figure(fig);hold on;
end

plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', [1,1,1]);
% plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');

num = PU.Num;
count = 0;
count2 = 0;
dim = PU.Set(1).Dim-2;

for p = 1:PU.Num;
    count = count + 1;
    if count >= PU.Num/100
        count2 = count2 + 1;
        disp("searching over "+num2str(count2)+ "/100");
        count = 0;
        drawnow;
        pause(0.001);
    end
    
    pp = PU.Set(p).slice(1:dim, x);
    if pp.isEmptySet
%         disp(['EmptySet at iteration ', num2str(p)]);
        continue
    end
    plot(pp, 'color', [150, 150, 255]/255, 'alpha',1,'edgealpha',0);
%     plot(pp, 'color', [0 0 1], 'alpha',0, 'linewidth',2);
%     plot(pp, 'color', 'g', 'alpha',1,'edgealpha',1);
end
