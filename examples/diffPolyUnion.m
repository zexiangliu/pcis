function P_diff = diffPolyUnion(P1,P2)
% compute P1 - P2
P_diff = PolyUnion();
for i = 1:P1.Num
    tmp = P1.Set(i);
    for j = 1:P2.Num
        tmp = tmp\P2.Set(j);
    end
    if ~isEmptySet(tmp)
        P_diff.add(tmp);
    end 
end

end