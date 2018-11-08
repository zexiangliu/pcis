function P = expand(varargin)
% Find controlled invariant set (CIS) C \subseteq S
% G is CIS to be expanded 
%
% Usage:
%   - P = expand(pwd0, S, G, rhoPre)
%   - P = expand(pwd0, S, G, rhoPre, 'plot_stuff')
%   - P = expand(pwd0, S, G, rhoPre, 'debug')
%   - P = expand(pwd0, S, G, rhoPre, 'max_iter', max_iter)
%   - P = expand(pwd0, S, G, rhoPre, 'plot_stuff','debug')
%
% Inputs:
%   plot_stuff: string, indicates that the system will plot figures of intermediate sets while operating.

%% Manage Inputs

pwd0 = varargin{1};
S = varargin{2};
G = varargin{3};
rhoPre = varargin{4};

%Assign defaults to the extra variables.
plot_stuff = false;
debug_flag = false;
max_iter = -1;

arg_ind = 5;
while arg_ind <= nargin
    switch varargin{arg_ind}
    case 'plot_stuff'
        plot_stuff = true;
        arg_ind = arg_ind + 1;
    case 'debug'
        debug_flag = true;
        arg_ind = arg_ind + 1;
    case 'max_iter'
        if (nargin - arg_ind < 1) || (~isa(varargin{arg_ind+1},'double'))
            error('Expect a numerical argument after the ''max_iter'' string')
        end
        max_iter = varargin{arg_ind+1};
        arg_ind = arg_ind + 2; 
    otherwise
        error(['Unrecognized string input: ' varargin{arg_ind} ])
    end
end

%% Main Function

converged = zeros(S.Num,1);
P = cell(S.Num,1);
for i = 1:S.Num
    P{i} = PolyUnion(G);
end

iter_num = 0;
while sum(converged) < S.Num   
    for i = 1:S.Num
        if converged(i) == 1
            continue
        end
        if plot_stuff
            subplot(221);hold on
            plot(P{i}(end).slice([1 4], [25 25]));
            set(gca,'Xdir','reverse','Ydir','reverse')
            axis([-1 5 -50 50]);
            xlabel('ye'); ylabel('h');
            title('vEgo = 25 m/s, vLead = 25 m/s')

            subplot(222);hold on
            plot(P{i}(end).slice([1 4], [30 20]));
            set(gca,'Xdir','reverse','Ydir','reverse')
            axis([-1 5 -50 50]);
            xlabel('ye'); ylabel('h');
            title('vEgo = 30 m/s, vLead = 20 m/s')

            subplot(223);hold on
            plot(P{i}(end).slice([1 4], [16 25]));
            set(gca,'Xdir','reverse','Ydir','reverse')
            axis([-1 5 -50 50]);
            xlabel('ye'); ylabel('h');
            title('vEgo = 16 m/s, vLead = 25 m/s')

            subplot(224);hold on
            plot(P{i}(end).slice([1 4], [25 0]));
            set(gca,'Xdir','reverse','Ydir','reverse')
            axis([-1 5 -50 50]);
            xlabel('ye'); ylabel('h');
            title('vEgo = 25 m/s, vLead = 0 m/s');
            drawnow;
        end
        
        pwd_pre = pwd0.pre(P{i}(end), rhoPre);
        temp_P_i = [];
        temp_P_i = [ temp_P_i uPoly_isx_Poly(pwd_pre,S.Set(i)) ];
        temp_P_i = PolyUnion(temp_P_i);
        temp_setMinus = setMinus3(temp_P_i,P{i});
        if temp_setMinus.Num == 0 %If there are no polyhedra in the result of the set minus. We have converged. Pre( P{i} ) \subseteq P{i}
           converged(i) = 1;
           if sum(converged) == S.Num
               return
           end
        else
            P{i} = temp_P_i;
        end

        %Update Loop Counter
        iter_num = iter_num + 1;
        
        if iter_num == max_iter
            disp('Maximum iterations reached.')
            return
        end
        
        if debug_flag
            disp(['Iteration #' num2str(iter_num) ' Complete.'])
            disp(' ')
        end

    end
end
        