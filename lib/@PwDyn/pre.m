function [preSets] = pre( pwd1 , X , rho )

	% Inputs:
	%	pwd1	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	% eps0 = 0.01;

	preSets = PolyUnion;
	for i=1:pwd1.num_region
	    new_poly = pwd1.reg_list{i}.intersect(pwd1.dyn_list{i}.pre_proj(X, rho));
	    % S = add1(S, new_poly);

	    figure;
		hold on;
		plot(pwd1.domain.projection([2 3]),'color','blue')
		plot(X.projection([2 3]),'color','magenta')
		plot(new_poly.projection([2 3]))
		axis([-5 5 -5 5])

		preSets.add(new_poly);
	end

end