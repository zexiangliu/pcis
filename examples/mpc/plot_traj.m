function plot_traj(ind_list,U_list1,U_list2,T_list,title_text,ylabel_text)
    
    shade(ind_list,U_list1,U_list2,T_list);hold on;
    bound(ind_list,U_list1,U_list2,T_list,'b');
    title(title_text, 'interpreter','latex')
    xlabel("$t ~ (s)$", 'interpreter','latex');
    ylabel(ylabel_text, 'interpreter','latex');
    set(gca,'fontsize',12);
    
end

