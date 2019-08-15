function [x, u, L, Vx, Vxx, cost,costi] = DDP(SIMULATE, COST, x0, u0, Op, singleStep)
% iLQG - solve the deterministic finite-horizon optimal control problem.
%
%        minimize sum_i CST(x(:,i),u(:,i)) + CST(x(:,end))
%            u
%        s.t.  x(:,i+1) = DYN(x(:,i),u(:,i))
%
% Inputs
% ======
% DYNCST - A combined dynamics and cost function. It is called in
% three different formats.
%
%  1) step:
%   [xnew,c] = DYNCST(x,u,i) is called during the forward pass. 
%   Here the state x and control u are vectors: size(x)==[n 1],  
%   size(u)==[m 1]. The cost c and time index i are scalars.
%   If Op.parallel==true (the default) then DYNCST(x,u,i) is be 
%   assumed to accept vectorized inputs: size(x,2)==size(u,2)==K
%  
%  2) final:
%   [~,cnew] = DYNCST(x,nan) is called at the end the forward pass to compute
%   the final cost. The nans indicate that no controls are applied.
%  
%  3) derivatives:
%   [~,~,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = DYNCST(x,u,I) computes the
%   derivatives along a trajectory. In this case size(x)==[n N+1] where N
%   is the trajectory length. size(u)==[m N+1] with NaNs in the last column
%   to indicate final-cost. The time indexes are I=(1:N).
%   Dimensions match the variable names e.g. size(fxu)==[n n m N+1]
%   note that the last temporal element N+1 is ignored for all tensors
%   except cx and cxx, the final-cost derivatives.
%
% x0 - The initial state from which to solve the control problem. 
% Should be a column vector. If a pre-rolled trajectory is available
% then size(x0)==[n N+1] can be provided and Op.cost set accordingly.
%
% u0 - The initial control sequence. A matrix of size(u0)==[m N]
% where m is the dimension of the control and N is the number of state
% transitions. 
%
%
% Op - optional parameters, see below
%
% Outputs
% =======
% x - the optimal state trajectory found by the algorithm.
%     size(x)==[n N+1]
%
% u - the optimal open-loop control sequence.
%     size(u)==[m N]
%
% L - the optimal closed loop control gains. These gains multiply the
%     deviation of a simulated trajectory from the nominal trajectory x.
%     size(L)==[m n N]
%
% Vx - the gradient of the cost-to-go. size(Vx)==[n N+1]
%
% Vxx - the Hessian of the cost-to-go. size(Vxx)==[n n N+1]
%
% cost - the costs along the trajectory. size(cost)==[1 N+1]
%        the cost-to-go is V = fliplr(cumsum(fliplr(cost)))
%
% lambda - the final value of the regularization parameter
%
% trace - a trace of various convergence-related values. One row for each
%         iteration, the columns of trace are
%         [iter lambda alpha g_norm dcost z sum(cost) dlambda]
%         see below for details.
%
% timing - timing information
%
%
%
% BIBTeX:
%
% @INPROCEEDINGS{
% author={Tassa, Y. and Mansard, N. and Todorov, E.},
% booktitle={Robotics and Automation (ICRA), 2014 IEEE International Conference on},
% title={Control-Limited Differential Dynamic Programming},
% year={2014}, month={May}, doi={10.1109/ICRA.2014.6907001}}
    costi=[];


% --- initial sizes and controls
n   = size(x0, 1);          % dimension of state vector
m   = size(u0, 1);          % dimension of control vector
N   = size(u0, 2);          % number of state transitions
u   = u0;                   % initial control sequence

% --- initial trajectory
[x,u,cost]  = forward_pass(x0(:,1),u,[],[],[],1,SIMULATE,COST);

for iter = 1:150
        costi = [costi ,sum(cost)];

    display([int2str(iter) ': ' num2str(sum(cost(:)))]);
    %====== STEP 1: differentiate dynamics and cost along new trajectory
    [~,fx,fu,fxx,fxu,fuu] = SIMULATE([u nan(m,1)], 1:N+1);
    [~,cx,cu,cxx,cxu,cuu] = COST([u nan(m,1)], 1:N+1);
    
    %====== STEP 2: backward pass, compute optimal control law and cost-to-go
    lambda = 0.0001; % regularization
    regType = 1;
    [~, Vx, Vxx, l, L, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,regType,u);
    
    %====== STEP 3: line-search to find new control sequence, trajectory, cost
    alpha = .2;
    costnew = zeros(size(cost));
    flag = true;
    while flag
        [xnew,unew,costnew] = forward_pass(x0 ,u+l*alpha, L, x(:,1:N),[],1,SIMULATE,COST);
        Op.plotFn([x; xnew]);
        if sum(cost(:)) < sum(costnew(:))
            alpha = alpha / 2;
%             display('line search fail');
        else
            flag = false;
        end
    end    
    %====== STEP 4: accept step (or not), draw graphics, print status

    % accept changes
    deltacost = costnew-cost;
    u              = unew;
    x              = xnew;
    cost           = costnew;
    Op.plotFn(x);
    drawnow;
    
    if(deltacost<1e-5 & sum(dV.^2)<1e-5 | nargin > 5 & singleStep)
        break;
    end
end


function [xnew,unew,cnew] = forward_pass(x0,u,L,x,du,Alpha,SIMULATE,COST)
% parallel forward-pass (rollout)
% internally time is on the 3rd dimension, 
% to facillitate vectorized dynamics calls

n        = size(x0,1);
K        = length(Alpha);
K1       = ones(1,K); % useful for expansion
m        = size(u,1);
N        = size(u,2);

xnew        = zeros(n,K,N);
xnew(:,:,1) = x0(:,ones(1,K));
unew        = zeros(m,K,N);
cnew        = zeros(1,K,N+1);
for i = 1:N
    unew(:,:,i) = u(:,i*K1);
    
    if ~isempty(du)
        unew(:,:,i) = unew(:,:,i) + du(:,i)*Alpha;
    end    
    
    if ~isempty(L)
        dx          = xnew(:,:,i) - x(:,i*K1);
        unew(:,:,i) = unew(:,:,i) + L(:,:,i)*dx;
    end

    xnew(:,:,i+1) = SIMULATE(unew(:,:,i), i*K1);
end
cnew = COST(cat(3,unew,nan(m,K,1)));
assert(size(cnew,1) == 1 & size(cnew,2) == N+1);
% put the time dimension in the columns
xnew = permute(xnew, [1 3 2]);
unew = permute(unew, [1 3 2]);
cnew = permute(cnew, [1 3 2]);

function [diverge, Vx, Vxx, k, K, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,regType,lims,u)
% Perform the Ricatti-Mayne backward pass

% tensor multiplication for DDP terms
vectens = @(a,b) permute(sum(bsxfun(@times,a,b),1), [3 2 1]);

N  = size(cx,2);
n  = numel(cx)/N;
m  = numel(cu)/N;

cx    = reshape(cx,  [n N]);
cu    = reshape(cu,  [m N]);
cxx   = reshape(cxx, [n n N]);
cxu   = reshape(cxu, [n m N]);
cuu   = reshape(cuu, [m m N]);

k     = zeros(m,N-1);
K     = zeros(m,n,N-1);
Vx    = zeros(n,N);
Vxx   = zeros(n,n,N);
dV    = [0 0];

Vx(:,N)     = cx(:,N);
Vxx(:,:,N)  = cxx(:,:,N);

diverge  = 0;
for i = N-1:-1:1
    
    Qu  = cu(:,i)      + fu(:,:,i)'*Vx(:,i+1);
    Qx  = cx(:,i)      + fx(:,:,i)'*Vx(:,i+1);
    Qux = cxu(:,:,i)'  + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    if ~isempty(fxu)
        fxuVx = vectens(Vx(:,i+1),fxu(:,:,:,i));
        Qux   = Qux + fxuVx;
    end
    
    Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i);
    if ~isempty(fuu)
        fuuVx = vectens(Vx(:,i+1),fuu(:,:,:,i));
        Quu   = Quu + fuuVx;
    end
    
    Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    if ~isempty(fxx)
        Qxx = Qxx + vectens(Vx(:,i+1),fxx(:,:,:,i));
    end
    
    Vxx_reg = (Vxx(:,:,i+1) + lambda*eye(n)*(regType == 2));
    
    Qux_reg = cxu(:,:,i)'   + fu(:,:,i)'*Vxx_reg*fx(:,:,i);
    if ~isempty(fxu)
        Qux_reg = Qux_reg + fxuVx;
    end
    
    QuuF = cuu(:,:,i)  + fu(:,:,i)'*Vxx_reg*fu(:,:,i) + lambda*eye(m)*(regType == 1);
    
    if ~isempty(fuu)
        QuuF = QuuF + fuuVx;
    end
    
    % no control limits: Cholesky decomposition, check for non-PD
    [R,d] = chol(QuuF);
    if d ~= 0
        diverge  = i;
        return;
    end

    % find control law
    kK = -R\(R'\[Qu Qux_reg]);
    k_i = kK(:,1);
    K_i = kK(:,2:n+1);
        
    
    % update cost-to-go approximation
    dV          = dV + [k_i'*Qu  .5*k_i'*Quu*k_i];
    Vx(:,i)     = Qx  + K_i'*Quu*k_i + K_i'*Qu  + Qux'*k_i;
    Vxx(:,:,i)  = Qxx + K_i'*Quu*K_i + K_i'*Qux + Qux'*K_i;
    Vxx(:,:,i)  = .5*(Vxx(:,:,i) + Vxx(:,:,i)');
    
    % save controls/gains
    k(:,i)      = k_i;
    K(:,:,i)    = K_i;
end