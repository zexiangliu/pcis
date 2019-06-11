function [Ct,log] = win_always_rho_inv(dyn, C0, rho_var, show_plot, verbose, maxiter)
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
    rho_var = @(n) 0;
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
  
  if nargout == 2
    log = struct();
    log.num_cons = [];
    log.ball = [];
    log.time = [];
  end
  
  C = Polyhedron('A', zeros(1,dyn.nx), 'b', 1);
  Ct = C0;
  iter = 0;

  tic;

  if show_plot
    figure; clf
  end
   
  mask = ones(length(C.b),1);
  Xb= shrink_rho(C,rho_var(size(C.A,1)),mask);
%   Xb = shrink_rho2(C,rho_var(size(C.A,1)),mask);
  cc_old = inf;
  while not (Xb <= Ct) && iter <= maxiter
    if show_plot
%       plot(Ct, 'alpha', 0.4); 
      plot(C.projection([1,2,3]))
      drawnow
    end
    
    % figure out the stable constraints
    mask = sum((abs(C.A*Ct.A') - abs(C.b-Ct.b')) >= 1-1e-10,1)' == 1;
    mask = ~mask;
    Xb = shrink_rho(Ct,rho_var(size(Ct.A,1)),mask);
%     Xb = shrink_rho2(Ct,rho_var(size(Ct.A,1)),mask);
    
    [Cpre] = dyn.pre_pure(Xb);
    if isEmptySet(Cpre)
      if verbose
        disp('returned empty')
      end
      Ct = Cpre;
      break;
    end
    
    C = Ct;
    Ct = intersect(Cpre, C);
%     Ct = intersect(Cpre, Xb);
%     Ct = intersect(Cpre, C0);
%     Ct = minHRep(Ct);
%     cc = Ct.chebyCenter;

%     if cc_old < cc.r
%     if ~C.contains(Ct)
%          % to guarantee the decreasing sequence
%         Ct = intersect(Cpre, C);
% %         cc = Ct.chebyCenter;
%     end
    Ct = minHRep2(Ct);
%     Ct = minHRep(Ct);

    cc = Ct.chebyCenter;
%     cc_old = cc.r;
    
    time=toc;

    iter = iter+1;
    if verbose
      disp(sprintf('iteration %d, %d ineqs, ball %f, time %f', ...
                 iter, size(Ct.A,1), cc.r, time));
             
      if nargout == 2
          log.ball(end+1) = cc.r;
          log.num_cons(end+1) = size(Ct.A,1);
          log.time(end+1) = time;
      end
    end
  end
  
  % sanity check in case of numerical issues
  Ct_pre = dyn.pre_pure(Ct);
  
  if ~(Ct <= Ct_pre)
      disp("verification fails. output conservative result.");
      Ct = Xb;
      Ct_pre = dyn.pre_pure(Ct);
      if ~(Ct <= Ct_pre)
          disp("verification of conservative result fails. The output is not trustful.");
      end
  end
  

  if verbose && ~isEmptySet(Ct)
    disp(sprintf('finished with nonempty set after %d iterations!', iter))
  end
end
  
function new_X = shrink_rho(X,rho,mask)
    A = X.A;
    b = X.b;
    
    % way 1: absolute shrink
    rho = sqrt(sum(A.^2,2))*rho;
    
    idx = mask==1;
    len = sum(idx);
    mask(idx) = rand(len,1)>= 0.8;
    b = b - mask.*rho;
%     % way 2: relative shrink
%     b = b - abs(b)*rho;
    
    new_X = Polyhedron('A',A,'b',b);
end

function new_X = shrink_rho2(X,rho,mask)
    A = X.A;
    b = X.b;
    
    cov1 = abs(A*A') > 1-0.01;
    cov2 = abs(b-b') < 0.3;
    
    filter = cov1 & cov2;
    row_rate = sum(filter,1);
%     row_rate(~mask)= 0;
    n = length(row_rate);
    
    check_list = false(n,1);
    mask2 = check_list;
    for i = 1:n
        if ~mask(i) %|| check_list(i)
            continue;
        end
        rate = row_rate(i);
        group_rate = row_rate(filter(:,1));
        if all(rate>=group_rate) && rate > 1
            mask2(i) = 1;
            check_list(filter(:,1))=true;
        end
    end
    
    rho = sqrt(sum(A.^2,2))*rho;
    
%     idx = mask2==1;
%     len = sum(idx);
%     mask2(idx) = rand(len,1)>= 0.8;
    b = b - mask2.*rho;
%     % way 2: relative shrink
%     b = b - abs(b)*rho;
    
    new_X = Polyhedron('A',A,'b',b);
end