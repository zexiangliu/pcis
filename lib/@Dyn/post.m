function X1 = post( varargin )
	%Description:
	%This function computes the post of the initial state set X0.
	%	X1 = A X0 + B U + F
	%
	%Usage:
	%X1 = d.post( X0 )
	%X1 = d.post( X0 , 'U' , U )

	%% Input Processing
	if nargin < 2
		error('post() requires at least two arguments!')
	end

	d = varargin{1};
	X0 = varargin{2};

	arg_idx = 3;
	while arg_idx <= nargin
		switch varargin{arg_idx}
			case 'U'
				U = varargin{arg_idx+1};
				arg_idx = arg_idx + 2;
			otherwise
				error(['Unexpected input to post ' varargin{arg_idx} '.' ] )
		end
	end

	if ~X0.hasVRep
		X0.computeVRep;
	end

	%Problem areas.
	if ~isempty(d.Ap) || ~isempty(d.Ad) || ~isempty(d.Ev) || ~isempty(d.Ew)
		error('This function does not currently support dynamics with : ''Ap'',''Ad'',''Ev'',''Ew'' ')
	end

	%% Constants

	n_x = d.nx();
	n_u = d.nu();

	if ~exist('U')
		d.XU.computeVRep;
		U = d.XU.projection(n_x+[1:n_u]);
	end

	%% Algorithm

	X1 = d.A * X0 + d.B * U + d.F;

end