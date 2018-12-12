function new_V = merge_cvx(V)
    
    tmp_V =V.Set(1);
    new_V = [];
    for i = 2:V.Num
        if  isNeighbor(tmp_V,V.Set(i))
            tmp_V = merge_in(tmp_V,V.Set(i));
            tmp_V.minHRep;
        else
            new_V = [new_V tmp_V];
            tmp_V = V.Set(i);
        end
    end
    new_V = PolyUnion([new_V tmp_V]);
end