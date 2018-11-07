function [preSets] = pre_int_xu(pwd1, pwd2, X, rho, regions, dyns_id, isParallel)
    % return the X-U pre of X (without projection to X space)
    % it is for input extraction
	% Usage:
	%	preSets = pre_int_uv(pwd1 , pwd2, X , rho)
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
                    % lift the rigion to X-U Space
                    tmp_inter = Polyhedron('H',[tmp_inter.A ...
                                    zeros(size(tmp_inter.A,1),pwd1.m)...
                                    tmp_inter.b]);
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
    
        new_poly = cell(num_reg,1);
        parfor i=1:num_reg
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            new_poly{i} = IntersectPolyUnion(regions{i},...
                pwd1.dyn_list{dyn_id1}.preIntUnion_xu(...
                pwd2.dyn_list{dyn_id2},X, rho));
        end

        for i = 1:num_reg
            for j = 1:new_poly{i}.Num
                preSets.add(new_poly{i}.Set(j));
            end
        end
        % reduce if too many polyhedron
        if(preSets.Num >= MaxNum)
            preSets.reduce;
        end
    else
        for i=1:num_reg
            dyn_id1 = dyns_id{i}(1);
            dyn_id2 = dyns_id{i}(2);
            new_poly = IntersectPolyUnion(regions{i},...
                pwd1.dyn_list{dyn_id1}.preIntUnion(...
                pwd2.dyn_list{dyn_id2},X, rho));
            
            for j = 1:new_poly.Num
                preSets.add(new_poly.Set(j));
            end
            
%             % reduce if too many polyhedron
%             if(preSets.Num >= MaxNum)
%                 preSets.reduce;
%                 MaxNum = max(MaxNum,preSets.Num*2);
%             end
        end
        
        % reduce if too many polyhedron
        if(preSets.Num >= MaxNum)
            preSets.reduce;
        end
    end
end