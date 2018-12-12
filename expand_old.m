function C = expand(dyn, S, G, rhoPre)
% Find controlled invariant set (CIS) C \subseteq S
% G is CIS to be expanded 
% S is the safe set
plot_stuff = 1;
converged = 0;
P = cell(S.Num+2,1);
for i = 1:S.Num
    P{i} = PolyUnion();
end
% all pre before a step
P{end-1} = PolyUnion();
% all pre after a step
P{end} = PolyUnion();
% new pres
new_pre = PolyUnion(G);
new_pre_Next = PolyUnion(G);

t = 0;
Gi = G;
figure;hold on;
while sum(converged) < S.Num  
    disp(['Iteration ', num2str(t)]); t = t+1;
    for i = 1:S.Num
        p = pre(dyn, new_pre, rhoPre);
        p = IntersectPolyUnion(p, S.Set(i));
%         p.minHRep;
        if p.Num > 0
            P{i} = PolyUnion([P{i}.Set p.Set]);
            P{end} = PolyUnion([P{end}.Set P{i}.Set]);
            new_pre_Next = PolyUnion([new_pre_Next.Set p.Set]);
        end
    end

    if plot_stuff
        plot(P{end});
        drawnow;
    end
    if mod(t,10) == 0
        Pminus = setMinus3(P{end}, P{end-1});
        if Pminus.Num < 1
            converged = 1;
        end
    end
    new_pre_Next.reduce();
    new_pre = new_pre_Next;
%     new_pre.merge();
%     Gi = P{end};
%     Gi.reduce;
%     Gi.merge;
end
        