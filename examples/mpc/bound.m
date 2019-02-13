function bound(ind_list,U_list1,U_list2,T_list,color)
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
       plot([t1,t2],U_list1(i:i+1),'-','color',color,'linewidth',lw);
       plot([t1,t2],U_list2(i:i+1),'-','color',color,'linewidth',lw);
    end
end