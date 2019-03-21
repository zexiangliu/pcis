function [Ct] = win_always_rho(dyn, C0, rho, show_plot, verbose, maxiter)
% win_always: compute set C ⊂ C0 such that
%   
%  C ⊂ Pre(C) + Ball(rho)
% 
% which is a sufficient condition for controlled invariance
%
% Reference: Rungger, M., & Tabuada, P. (2017). Computing Robust 
% Controlled Invariant Sets of Linear Systems. IEEE Trans. on 
% Automatic Control, dx.doi.org/10.1109/TAC.2017.2672859

  if nargin < 3
    rho = 0;
  end

  if nargin < 4
    show_plot = 0;
  end

  if nargin < 5
    verbose = 0;
  end
    
  if nargin < 6
      maxiter = inf;
  end
  
  C = Polyhedron('A', zeros(1,dyn.nx), 'b', 1);
  Ct = C0;
  iter = 0;

  tic;

  if show_plot
    figure; clf
  end

  while not (shrink_rho(C,rho) <= Ct) && iter <= maxiter
    C = Ct;

    if show_plot
      plot(C, 'alpha', 0.4); 
%       plot(C.projection([1,2,3]))
      drawnow
    end

    Cpre = dyn.pre_rho(C, rho);
    if isEmptySet(Cpre)
      if verbose
        disp('returned empty')
      end
      Ct = Cpre;
      break;
    end

    Ct = intersect(Cpre, C0);
    Ct = minHRep(Ct);

    cc = Ct.chebyCenter;
    time=toc;

    iter = iter+1;
    if verbose
      disp(sprintf('iteration %d, %d ineqs, ball %f, time %f', ...
                 iter, size(Ct.A,1), cc.r, time));
    end
  end

  if verbose && ~isEmptySet(Ct)
    disp(sprintf('finished with nonempty set after %d iterations!', iter))
  end
end
  
function new_X = shrink_rho(X,rho)
    A = X.A;
    b = X.b;
    
    % way 1: absolute shrink
    rho = sqrt(sum(A.^2,2))*rho;
    b = b - rho;
%     % way 2: relative shrink
%     b = b - abs(b)*rho;
    
    new_X = Polyhedron('A',A,'b',b);
end