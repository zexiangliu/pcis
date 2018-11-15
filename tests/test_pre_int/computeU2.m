function u = computeU2(x,u, PU)
% x is a point, PU is polyUnion

num = PU.Num;
count = 0;
count2 = 0;

u1_max = -3;
u2_max = -1.8;
u1_min = 3;
u2_min = 1.8;

for p = 1:PU.Num
    count = count + 1;
    if count >= PU.Num/10
        count2 = count2 + 1;
        if(PU.Num > 1e4)
            disp("searching over "+num2str(count2)+ "/10");
        end
        count = 0;
    end
    
    if ~PU.Set(p).contains([x;u(:,1)])&&~PU.Set(p).contains([x;u(:,2)])&&...
            ~PU.Set(p).contains([x;u(:,3)])&&~PU.Set(p).contains([x;u(:,4)])&&...
            ~PU.Set(p).contains([x;u(:,5)])
        continue;
    end
    
    pp = PU.Set(p).slice([1 2 3 4], x);
%     if pp.isEmptySet
% %         disp(['EmptySet at iteration ', num2str(p)]);
%         continue
%     end
    
    V = pp.V;
    u1_max = max(u1_max,max(V(:,1)));
    u1_min = min(u1_min, min(V(:,1)));
    
    u2_max = max(u2_max,max(V(:,2)));
    u2_min = min(u2_min,min(V(:,2)));
    
    if(abs(u1_max - 3)<= 0.1 && abs(u1_min + 3)<= 0.1 ...
            && abs(u2_max - 1.8)<= 0.1 && abs(u2_min + 1.8)<= 0.1)
        break;
    end
    
end

if u1_min <= u1_max && u2_min <= u2_max
    u = [u1_min u1_max;u2_min u2_max];
else
    u = [0 0 ; 0 0];
end