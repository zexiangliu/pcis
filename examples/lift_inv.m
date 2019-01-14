function seed = lift_inv(C)
% lift the 3D inv set from bnd vel model to 4D seed for known intentions
con = constants_tri();

seed = PolyUnion();

for i = 1:C.Num
    old_poly = C.Set(i);
    old_A = old_poly.A;
    old_b = old_poly.b;
    m = size(old_A,1);
    A = zeros(m+2,4);
    b = zeros(m+2,1);
    A(1:m,1:3) = old_A;
    b(1:m) = old_b;
    A(m+1,4) = 1;
    b(m+1) = con.vL_max;
    A(m+2,4) = -1;
    b(m+2) = -con.vL_min;
    new_poly = Polyhedron('A',A,'b',b);
    seed.add(new_poly);
end

end

