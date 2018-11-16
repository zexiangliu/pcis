v = V.Set(20).V;

alpha = rand(1,size(v,1));
alpha = alpha/sum(alpha);

% x = (alpha*v(:,1:4))';
x = v(30,:)';
u_range = preXU.slice([1,2,3,4],x);
plot(u_range)
xlabel("a_{e,x}");
ylabel("v_{e,y}");
title("Slice input at state ["+num2str(x(1))+" "+num2str(x(2))+...
    " "+num2str(x(3))+" "+num2str(x(4))+"]");