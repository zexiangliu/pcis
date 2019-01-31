function plot_traj(ind_list,U_list1,U_list2,T_list,title_text,ylabel_text)
    
    shade(ind_list,U_list1,U_list2,T_list);
    bound(ind_list,U_list1,U_list2,T_list);
    title(title_text, 'interpreter','latex')
    xlabel("$t$", 'interpreter','latex');
    ylabel(ylabel_text, 'interpreter','latex');
    set(gca,'fontsize',12);
    
end

function shade(ind_list,U_list1,U_list2,T_list)
    num = length(ind_list);
    for i = 1:num-1
        ind1 = ind_list(i);

        ind2 = ind_list(i+1);
        if ind2 - ind1 ~=1
            continue;
        end
        t1 = T_list(ind1);
        t2 = T_list(ind2);
        v1_1 = U_list1(i);
        v1_2 = U_list1(i+1);
        v2_1 = U_list2(i);
        v2_2 = U_list2(i+1);
        V = [t1 v1_1;t1 v2_1;t2 v2_2;t2 v1_2];
        F = [1 2 3 4];
        patch('Faces',F,'Vertices',V,'FaceColor','b','EdgeColor','none','FaceAlpha',.1);
    end
end
function bound(ind_list,U_list1,U_list2,T_list)
    lw = 1.5;
    num = length(ind_list);
    for i = 1:num-1
       ind1 = ind_list(i);
       ind2 = ind_list(i+1);
       if ind2 - ind1 ~= 1
           continue;
       end
       t1 = T_list(ind1);
       t2 = T_list(ind2);
       try
           plot([t1,t2],U_list1(i:i+1),'-b','linewidth',lw);
           plot([t1,t2],U_list2(i:i+1),'-b','linewidth',lw);
       catch
           keyboard();
       end
    end
end