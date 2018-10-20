function [preSets] = pre( varargin )

	% Usage:
	%	preSets = pre(pwd1 , X , rho)
	%	preSets = pre(pwd1 , X , rho , 'plot_stuff')
	% Inputs:
	%	pwd1	- The piecewise affine dynamics we are using.
	%	X 		- Target Set. The set for which we are calculating the pre.
	%
	% Outputs:
	%	preSets - An array of polyhedra. All possible polyhedra that could result in the target states X.

	%% Input Processing

	pwd1 = varargin{1};
	X = varargin{2};
	rho = varargin{3};

	plot_flag = false;

	if nargin >= 4
		for ind = 4:nargin
			switch varargin{ind}
				case 'plot_stuff'
					plot_flag = true;
				otherwise
					error(['Unrecognized input: ' varargin{ind} ])
			end
		end
	end


	% eps0 = 0.01;

	preSets = PolyUnion;
	for i=1:pwd1.num_region
	    new_poly = pwd1.reg_list{i}.intersect(pwd1.dyn_list{i}.pre_proj(X, rho));
	    % S = add1(S, new_poly);

	    if plot_flag
		    figure;
			hold on;
			plot(pwd1.domain.projection([2 3]),'color','blue')
			plot(X.projection([2 3]),'color','magenta')
			plot(new_poly.projection([2 3]))
			axis([-5 5 -5 5])
		end
		
		preSets.add(new_poly);
	end

end