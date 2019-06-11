h_max = 5000;

box = Polyhedron('UB', [con.v_max;   con.y_max;      h_max;      h_max],...
                'LB', [con.v_min;   con.y_min;      -h_max;  -h_max]);
            
CIS2 = IntersectPolyUnion(CIS,box);

%%
CIS_proj = projectionPolyUnion(CIS2,[1,2,3]);
plot(CIS_proj,'alpha',0.5,'color','g');










%%

x1 = [25,-20]


C1 = CIS2.slice([1,4],x1)

plot(C1,'color','r','edgealpha',0)
axis([-1,2.8,-50,50])
set(gca,'Xdir','reverse','Ydir','reverse')
title("ego car velocity: "+num2str(x1(1))+"m/s, h_2: "+num2str(x1(2))+"m");