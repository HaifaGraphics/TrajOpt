function [x, u, cost,costi] = Newton(DYNCST, x0, u0, Op)
    costi=[];
    % --- initial sizes and controls
    n   = size(x0, 1);          % dimension of state vector
    m   = size(u0, 1);          % dimension of control vector
    N   = size(u0, 2);          % number of state transitions
    u   = u0;                   % initial control sequence

    % --- initial trajectory
    [x,cost]  = forward_pass(x0,u,DYNCST);

    for iter = 1:100
        costi = [costi ,sum(cost)];

        display([int2str(iter) ': ' num2str(sum(cost(:)))]);
        %====== STEP 1: compute Hessian and derivative
        [~,~,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = DYNCST(x, [u nan(m,1)], 1:N+1);
        cu = cu(:,1:end-1);
        temp = cellfun(@sparse,  num2cell(fx,[1,2]), 'uni',0);
        fx=blkdiag(temp{1:end-1});
        A = spdiags(ones(size(fx,1)+8,1),0,size(fx,1)+4,size(fx,1)+4);
        A(5:end,1:end-4)=A(5:end,1:end-4)-fx;
        fx = A;
        
        temp = cellfun(@sparse,  num2cell(fu,[1,2]), 'uni',0  );
        fu=-blkdiag(temp{1:end-1});
        fu = [zeros(4,size(fu,2)); fu];
        S=-fx\fu;
        dcdu = S' * cx(:)  + cu(:);
        % verification
        eps = 1e-6;
        du = eps*(rand(size(u))-0.5);
        dx1 = reshape(S*du(:),4,[]);
        [xp,costp]  = forward_pass(x0,u+du,DYNCST);
        dx2 = xp-x;
        dc1 = dcdu'*du(:);
        dc2 = sum(costp(:)-cost(:));
        
        temp = cellfun(@sparse,  num2cell(cuu,[1,2]), 'uni',0  );
        cuu=blkdiag(temp{1:end-1});

        temp = cellfun(@sparse,  num2cell(cxx,[1,2]), 'uni',0  );
        cxx=blkdiag(temp{1:end});

        temp = cellfun(@sparse,  num2cell(cxu,[1,2]), 'uni',0  );
        cxu=blkdiag(temp{1:end-1});
        
        H = S'*cxx*S + cuu;
        H=H+0.0*sparse(eye(size(H)));

        %====== STEP 2: Line search
        p = reshape(-H\dcdu,2,[]);
%         p = reshape(-dcdu,2,[]);
        if(p(:)'*dcdu>0)
            disp('not a search direction');
        end
        s=eig(H'+H);
        if(any(s<0))
            disp('H not PSD');
        end
        
        alpha = 1;
        costnew = zeros(size(cost));
        flag = true;
        while flag
            [xnew,costnew] = forward_pass(x0 ,u+alpha*p,DYNCST);
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
        u              = u+alpha*p;
        x              = xnew;
        cost           = costnew;
        Op.plotFn(x);
        drawnow;
        if(deltacost<1e-5 & sum(dcdu.^2)<1e-5)
            break;
        end
    end
    % check control sequence legality
    if ~isempty(Op.lims)
        switch numel(Op.lims)
            case 0
            case 2*m
                Op.lims = sort(Op.lims,2);
            case 2
                Op.lims = ones(m,1)*sort(Op.lims(:))';
            case m
                Op.lims = Op.lims(:)*[-1 1];
            otherwise
                error('limits are of the wrong size')
        end
            

        if ~all(u > Op.lims(:, 1*ones(1,N)) & u < Op.lims(:, 2*ones(1,N)), 'all')
            display(u);
            error('Illegal control sequence')
        else
            %Disp max control utilization% metric to ensure we aren't restricting
            %ourselves from using all of the available control bandwidth
            display('Control Utilization%')
            (max(u,[],2) - min(u,[],2)) ./ (Op.lims(:,2) - Op.lims(:,1)) .* 100
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
        [x(:,i+1), cost(:,i)]  = DYNCST(x(:,i), u(:,i));
    end
    [~, cost(:,N+1)] = DYNCST(x(:,N+1),[nan;nan]);

end