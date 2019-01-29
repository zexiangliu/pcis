function visual(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end

color = 'c';
subplot(221);hold on
    p = Polyhedron('ub', [5, 50], 'lb', [-1,-50]);
    p2 = IntersectPolyUnion(p,V.slice([1 4], [25 25]));
    plot(p2, 'color', color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('y_e'); ylabel('h');
    title('v_{e,x} = 25 m/s, v_{L,x} = 25 m/s')

    subplot(222);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [30 20]));
    plot(p2, 'color', color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('y_e'); ylabel('h');
    title('v_{e,x} = 30 m/s, v_{L,x} = 20 m/s')

    subplot(223);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [16 25]));
    plot(p2, 'color', color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('y_e'); ylabel('h');
    title('v_{e,x} = 16 m/s, v_{L,x} = 25 m/s')

    subplot(224);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [25 0]));
    plot(p2, 'color', color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('y_e'); ylabel('h');
    title('v_{e,x} = 25 m/s, v_{L,x} = 0 m/s');
    drawnow;
end