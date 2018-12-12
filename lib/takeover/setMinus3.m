function P3 = setMinus3(P1, P2)
% returns the set difference P1\P2 where
% P1,P2 are either Polyhedron or PolyUnion objects
% P3 is a PolyUnion object
% P3 = union_{p1\in P1}(intersect_{p2 \in P2} p1\p2)

% make sure P1/P2 are PolyUnion
if ~isa(P1, 'PolyUnion')
    P1 = PolyUnion(P1);
end
if ~isa(P2, 'PolyUnion')
    P2 = PolyUnion(P2);
end

P3 = PolyUnion;

for p1 = 1:P1.Num
    p1P2 = P1.Set(p1); % p1\P2
    for p2 = 1:P2.Num
        p1p2 = mldivide(P1.Set(p1), P2.Set(p2)); % p1\p2
        p1P2 = IntersectPolyUnion(p1P2, p1p2, 20);
    end
    if ~isa(p1P2, 'PolyUnion')
        p1P2 = PolyUnion(p1P2);
    end
    if p1P2.Num >= 1
        P3.add(p1P2.Set);
    end
end