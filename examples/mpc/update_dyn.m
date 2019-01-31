function [x_new, Iv] = update_dyn(x,u,w,UnSafe,con, intention)
    
    if intention == "agg"
        if nargin == 2
            [x_new, Iv] = update_agg(x,u,w,UnSafe,con);
        else
            x_new = update_agg(x,u,w,UnSafe,con);
        end
    elseif intention == "cau"
        if nargin == 2
            [x_new, Iv] = update_cau(x,u,w,UnSafe,con);
        else
            x_new = update_cau(x,u,w,UnSafe,con);
        end
    else
        error("intention has to be 'cau' or 'agg'!");
    end
    
end