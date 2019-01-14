function visual2(fig,C)
    figure(fig);
    
    proj = projectionPolyUnion(C,[1,2,3]);
    outerBox = Polyhedron("H", [0 0 1 50;0 0 -1 50])
    proj = IntersectPolyUnion(proj,outerBox);
    plot(proj,'color','r');
end