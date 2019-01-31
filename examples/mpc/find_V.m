function [v1,v2] = find_V(u)
    
    u.reduce();
    if u.Num == 1
        sub_u = u.Set(1);
        V = sub_u.V;
        v1 = min(V);
        v2 = max(V);
        return;
    end
    v1 = inf;
    v2 = -inf;
    for i = 1:u.Num
        sub_u = u.Set(i);
        V = sub_u.V;
        v1 = min(min(V),v1);
        v2 = max(max(V),v2);
    end
end