function [safeU] = get_safe_inputs(intention,x,S,dyn_agg,dyn_cau)
%get_safe_inputs Given the intention, we generate the polyhedron of safe
%inputs of the ego vehicle in the takeover example.
%   Usage:
%
%   Inputs:
%       intention - A simple string
%                   Allowed: 'ac'
%
%       x - The current state of the system that we are evaluating.
%            Should be a 4d vector.
%
%       dyn_agg - PwDyn object. Represents aggressive lead car dynamics.
%
%       dyn_cau - PwDyn object. Represents cautious lead car dynamics.
%

%% Input Processing

if ~ischar(intention)
    error('First input (intention) must be a string.')
end

%% Algorithm

S_local = [];
for i = 1:S.Num
    if S.Set(i).contains(x)
        S_local = [S_local S.Set(i)];
    end
end
S_local = PolyUnion(S_local);

safeU = Polyhedron();
switch intention
    case 'ac'
        [~ , preXU ] = pre_int( dyn_agg , dyn_cau , S_local , 0, [], [], false, true );
        safeU = preXU.slice([1,2,3,4],x);
    otherwise
        error('Other intentions are not built in.')
end

end

