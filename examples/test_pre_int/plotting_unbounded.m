figure;
hold on;
for i=1:CIS_ac.Num 
pp = intersect(CIS_ac.Set(i).slice([1 4], [25 25]), Polyhedron('ub', [5 50], 'lb', [-1 -50]));
plot(pp, 'Color', 'b');
end