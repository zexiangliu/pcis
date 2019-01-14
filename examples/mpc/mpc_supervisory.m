function [u,U] = mpc_supervisory(x0, C, C_XU, con)
% the mpc controller under supervisory
% inputs: x0 --- current state
%         v_l --- lead car speed
%         C --- inv set in X space
%         C_XU --- inv set in XU space
    U = mpc_simple(x0(1:3),x0(4), con);
    u = U(:,1);
    dim = C.Set(1).Dim;
    if ~containsPolyUnion(C,x0(1:dim))
        warning("not in inv set.");
    elseif ~containsPolyUnion(C_XU, [x0(1:dim);u])
        
        % let's project u to the feasible input set
        proj_dist = inf;
        u_proj = [];
        for p = 1:C_XU.Num
            pp = C_XU.Set(p).slice(1:dim, x0(1:dim));
            if pp.isEmptySet
                continue
            else
                H = eye(2);
                f = -u;
                [u_tmp, fval, exitflag] = quadprog(H,f,pp.A,pp.b);
                if exitflag == 1
                    if fval < proj_dist
                        u_proj = u_tmp;
                    end
                end
            end
        end
        if ~isempty(u_proj)
            u = u_proj;
        end
    end
end