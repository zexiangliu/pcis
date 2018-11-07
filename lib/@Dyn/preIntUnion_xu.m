function X0 = preIntUnion_xu(dyn1, dyn2, X, rho)
% pre_int_ux for the input is a PolyUnion

%     dyn1.check();
%     dyn2.check();
    if nargin < 3
        rho = 0;
    end

    if isa(X, 'PolyUnion')	
        X0 = PolyUnion;
        for i=1:X.Num
            new_poly = pre_int_xu(dyn1, dyn2, X.Set(i), rho);
            X0.add(new_poly);
        end
    else
        X0 = pre_int_xu(dyn1, dyn2, X, rho);
    end
end