function visual(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end

r=50;
    
subplot(221);hold on
    p = V.slice([1 4], [25 25]);
    plot(IntersectPolyUnion(p,Polyhedron('UB', [5; r], 'LB', [-2; -r])));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1.2 3 -r r]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 25 m/s')

    subplot(222);hold on
    p = V.slice([1 4], [30 20]);
    plot(IntersectPolyUnion(p,Polyhedron('UB', [5; r], 'LB', [-2; -r])));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1.2 3 -r r]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 30 m/s, vLead = 20 m/s')

    subplot(223);hold on
    p = V.slice([1 4], [16 25]);
    plot(IntersectPolyUnion(p,Polyhedron('UB', [5; r], 'LB', [-2; -r])));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1.2 3 -r r]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 16 m/s, vLead = 25 m/s')

    subplot(224);hold on
    p = V.slice([1 4], [25 0]);
    plot(IntersectPolyUnion(p,Polyhedron('UB', [5; r], 'LB', [-2; -r])));
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1.2 3 -r r]);
    xlabel('ye'); ylabel('h');
    title('vEgo = 25 m/s, vLead = 0 m/s');
    drawnow;
end