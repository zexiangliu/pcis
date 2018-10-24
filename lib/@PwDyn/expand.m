function P = expand(varargin)
% Find controlled invariant set (CIS) C \subseteq S
% G is CIS to be expanded 
%
% Usage:
%   - P = expand(pwd0, S, G, rhoPre)
%   - P = expand(pwd0, S, G, rhoPre, 'plot_stuff')
%   - P = expand(pwd0, S, G, rhoPre, 'debug')
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

for arg_ind = 5:nargin
    switch varargin{arg_ind}
    case 'plot_stuff'
        plot_stuff = true;
    case 'debug'
        debug_flag = true;
    otherwise
        error(['Unrecognized string input: ' varargin{arg_ind} ])
    end
end

%% Main Function

converged = zeros(S.Num,1);
P = cell(S.Num,1);
for i = 1:S.Num
    P{i} = G;
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
        P{i} = [P{i} uPoly_isx_Poly(pwd_pre,S.Set(i))];
        if isEmptySet(mldivide(P{i}(end),P{i}(end-1)))
           converged(i) = 1;
           if sum(converged) == S.Num
               return
           end
        end

        %Update Loop Counter
        iter_num = iter_num + 1;
        if debug_flag
            disp(['Iteration #' num2str(iter_num) ' Complete.'])
            disp(' ')
        end

    end
end
        