function visual(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end
    
subplot(221);hold on
    p = Polyhedron('ub', [5, 50], 'lb', [-1,-50]);
    p2 = IntersectPolyUnion(p,V.slice([1 4], [25 25]));
    plot(p2, 'color', 'r');
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 25 m/s')

    subplot(222);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [30 20]));
    plot(p2, 'color', 'r');
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s, vLead = 20 m/s')

    subplot(223);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [16 25]));
    plot(p2, 'color', 'r');
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s, vLead = 25 m/s')

    subplot(224);hold on
    p2 = IntersectPolyUnion(p, V.slice([1 4], [25 0]));
    plot(p2, 'color', 'r');
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 5 -30 30]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 0 m/s');
    drawnow;
end