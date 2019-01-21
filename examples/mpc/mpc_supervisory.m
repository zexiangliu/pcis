function [u_s, u ,U, U_f] = mpc_supervisory(x0, C, C_XU, con)
% the mpc controller under supervisory
% inputs: x0 --- current state
%         v_l --- lead car speed
%         C --- inv set in X space
%         C_XU --- inv set in XU space
% outputs: u --- current input
%          U --- input sequence gotten by MPC
%          U_f --- polyunion, feasible inputs
    U = mpc_simple(x0(1:3),x0(4), con);
    u = U(:,1);
    dim = C.Set(1).Dim;
    if nargout == 4
        U_f = PolyUnion();
        flag_Uf = 1;
    else
        flag_Uf = 0;
    end
    u_s = u;
    if ~containsPolyUnion(C,x0(1:dim))
        error("not in inv set.");
        if flag_Uf
            U_f = get_input(C_XU,x0,dim);
        end
    elseif ~containsPolyUnion(C_XU, [x0(1:dim);u])
        % let's project u to the feasible input set
        proj_dist = inf;
        u_proj = [];
        U_f = get_input(C_XU,x0,dim);

        for p = 1:U_f.Num
            pp = U_f.Set(p);
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
            u_s = u_proj;
        end
    elseif flag_Uf
        U_f = get_input(C_XU,x0,dim);
    end
end