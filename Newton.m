function [x, u, L, Vx, Vxx, cost] = Newton(DYNCST, x0, u0, Op)

    % --- initial sizes and controls
    n   = size(x0, 1);          % dimension of state vector
    m   = size(u0, 1);          % dimension of control vector
    N   = size(u0, 2);          % number of state transitions
    u   = u0;                   % initial control sequence

    % --- initial trajectory
    [x,cost]  = forward_pass(x0,u,DYNCST);

    for iter = 1:100
        display([int2str(iter) ': ' num2str(sum(cost(:)))]);
        %====== STEP 1: differentiate dynamics and cost along new trajectory
        [~,~,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = DYNCST(x, [u nan(m,1)], 1:N+1);
        cu = cu(:,1:end-1);
        temp = cellfun(@sparse,  num2cell(fx,[1,2]), 'uni',0);
        fx_mat=blkdiag(temp{1:end-1});
        A = spdiags(ones(size(fx_mat,1)+8,1),0,size(fx_mat,1)+4,size(fx_mat,1)+4);
        A(5:end,1:end-4)=A(5:end,1:end-4)-fx_mat;
        fx_mat = A;
        
        temp = cellfun(@sparse,  num2cell(fu,[1,2]), 'uni',0  );
        fu_mat=-blkdiag(temp{1:end-1});
        fu_mat = [zeros(4,size(fu_mat,2)); fu_mat];
        S=-fx_mat\fu_mat;
        dcdu =cx(:)'* S + cu(:)';
        % verification
        eps = 1e-6;
        du = eps*rand(size(u));
        dx1 = reshape(S*du(:),4,[]);
        [xp,costp]  = forward_pass(x0,u+du,DYNCST);
        dx2 = xp-x;
        dc1 = dcdu*du(:);
        dc2 = sum(costp(:)-cost(:));
        
        %====== STEP 2: backward pass, compute optimal control law and cost-to-go
        lambda = 0.01; % regularization
        regType = 1;
        [~, Vx, Vxx, l, L, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,regType,u);

        %====== STEP 3: line-search to find new control sequence, trajectory, cost
        alpha = 1;
        costnew = zeros(size(cost));
        flag = true;
        while flag
            [xnew,unew,costnew] = forward_pass(x0 ,u+l*alpha, L, x(:,1:N),[],1,DYNCST,Op.lims);
            if sum(cost(:)) < sum(costnew(:))
                alpha = alpha / 2;
                display('line search fail');
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
        if(deltacost<1e-5 & sum(dV.^2)<1e-5)
            break;
        end
    end

end
function [x,cost] = forward_pass(x0,u,DYNCST)
    n = size(x0,1);
    N = size(u,2);

    x = zeros(n,N);
    cost = zeros(1,N+1);
    x(:,1)=x0;
    for i = 1:N
        [x(:,i+1), cost(:,:,i)]  = DYNCST(x(:,i), u(:,i));
    end
end