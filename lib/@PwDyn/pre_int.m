function [preSets] = pre_int(pwd1, pwd2, X, rho, regions, dyns_id)

	% Usage:
	%	preSets = pre_int(pwd1 , pwd2, X , rho)
	% Inputs:
	%	pwd1,pwd2	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	%% Input Processing
	
    if(nargin <= 4)
        regions = cell(pwd1.num_region*pwd2.num_region,1);
        dyns_id = cell(pwd1.num_region*pwd2.num_region,1);
        num_reg = 0;
    
        for i = 1:pwd1.num_region
            for j = 1:pwd2.num_region
                tmp_inter = intersect(pwd1.reg_list{i},pwd2.reg_list{j});
                if ~isEmptySet(tmp_inter) && tmp_inter.volume > 0
                    num_reg = num_reg + 1;
                    tmp_inter.minHRep;
                    regions{num_reg} = tmp_inter;
                    dyns_id{num_reg} = [i,j];
                end
            end
        end
    else
        num_reg = length(regions);
    end
    
	preSets = PolyUnion;
    MaxNum = 20;
	for i=1:num_reg
        dyn_id1 = dyns_id{i}(1);
        dyn_id2 = dyns_id{i}(2);
	    new_poly = IntersectPolyUnion(regions{i},...
            pwd1.dyn_list{dyn_id1}.preIntUnion(...
            pwd2.dyn_list{dyn_id2},X, rho));
        
% 		
        for j = 1:new_poly.Num
            preSets.add(new_poly.Set(j));
        end
        % reduce if too many polyhedron
        if(preSets.Num >= MaxNum)
            preSet.reduce;
            MaxNum = max(MaxNum,2*preSet.Num);
        end
	end

end