function visual3D(C,fig)
    if nargin == 1
        fig = figure;
    else
        figure(fig);
    end
    
    proj = projectionPolyUnion(C,[1,2,3]);
    outerBox = Polyhedron("H", [0 0 1 50;0 0 -1 50])
    proj = IntersectPolyUnion(proj,outerBox);
    plot(proj,'color','r');
end