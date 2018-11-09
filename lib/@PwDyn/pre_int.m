function [preSets,P_xu1,P_xu2] = pre_int(pwd1, pwd2, X, rho, regions, dyns_id, isParallel)

	% Usage:
	%	preSets = pre_int(pwd1 , pwd2, X , rho)
	% Inputs:
	%	pwd1,pwd2	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	%% Input Processing
	
    if(nargin <= 4 || isempty(regions) || isempty(dyns_id))
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
    
    if(nargin <= 6)
        isParallel = false;
    end
    
	preSets = PolyUnion;
    MaxNum = 20;
    
    if(isParallel)
        
        P_xu1 = cell(pwd1.num_region,1);
        P_xu2 = cell(pwd2.num_region,1);
        for i = 1:pwd1.num_region
            tmp_P_xu = preUnion_xu(pwd1.dyn_list{i},X,rho);
            P_xu1{i} = tmp_P_xu;
        end
        
        for i = 1:pwd2.num_region
            tmp_P_xu = preUnion_xu(pwd2.dyn_list{i},X,rho);
            P_xu2{i} = tmp_P_xu;
        end
        
        new_poly = cell(num_reg,1);
        parfor i=1:num_reg
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            P_inter = IntersectPolyUnion(P_xu1{dyn_id1},P_xu2{dyn_id2});
            new_poly{i} = IntersectPolyUnion(regions{i},...
                projectionPolyUnion(P_inter,1:pwd1.n));
        end
        
        preSets = [];
        for i = 1:num_reg
            preSets = [preSets new_poly{i}.Set];
        end
        
        preSets = PolyUnion(preSets);
        % reduce if too many polyhedron
        if(preSets.Num >= MaxNum)
            preSets.reduce;
        end
    else
        P_xu1 = cell(pwd1.num_region,1);
        P_xu2 = cell(pwd2.num_region,1);
        for i = 1:pwd1.num_region
            tmp_P_xu = preUnion_xu(pwd1.dyn_list{i},X,rho);
            P_xu1{i} = tmp_P_xu;
        end
        
        for i = 1:pwd2.num_region
            tmp_P_xu = preUnion_xu(pwd2.dyn_list{i},X,rho);
            P_xu2{i} = tmp_P_xu;
        end
        
        preSets = [];
        for i=1:num_reg
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            P_inter = IntersectPolyUnion(P_xu1{dyn_id1},P_xu2{dyn_id2});
            new_poly = IntersectPolyUnion(regions{i},...
                projectionPolyUnion(P_inter,1:pwd1.n));
            
            preSets = [preSets new_poly.Set];
            
%             % reduce if too many polyhedron
%             if(preSets.Num >= MaxNum)
%                 preSets.reduce;
%                 MaxNum = max(MaxNum,preSets.Num*2);
%             end
        end
        
        preSets = PolyUnion(preSets);
        % reduce if too many polyhedron
        if(preSets.Num >= MaxNum)
            preSets.reduce;
        end
    end
end