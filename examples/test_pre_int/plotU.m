function plotU(x, PU)
% x is a point, PU is polyUnion

figure;hold on;
plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');

num = PU.Num;
count = 0;
count2 = 0;

for p = 1:PU.Num;
    count = count + 1;
    if count >= PU.Num/100
        count2 = count2 + 1;
        disp("searching over "+num2str(count2)+ "/100");
        count = 0;
        drawnow;
        pause(0.001);
    end
    
    pp = PU.Set(p).slice([1 2 3 4], x);
    if pp.isEmptySet
%         disp(['EmptySet at iteration ', num2str(p)]);
        continue
    end
    plot(pp, 'color', 'g');
end
