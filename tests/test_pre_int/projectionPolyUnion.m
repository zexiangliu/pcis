function proj = projectionPolyUnion(P,dims)
    proj = PolyUnion;
    for i = 1:P.Num
        proj.add(P.Set(i).projection(dims));
    end

end