function plot_overriding(override_list, U_list,T_list)
wl = 3.5;   
step = length(override_list)-1;
    for i = 1:step
        ind1 = override_list(i);
        ind2 = override_list(i+1);
        if ind2 - ind1 ~= 1
            continue;
        end
        t1 = T_list(ind1);
        t2 = T_list(ind2);
        plot([t1,t2],U_list(ind1:ind2),'-g','linewidth',wl);hold on;
%         plot([t1,t2],UMPC_list(i:i+1),'-r','linewidth',wl);
    end
end