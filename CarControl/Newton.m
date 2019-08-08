function [x, u, cost,costi] = Newton(SIMULATE, COST, x0, u0, Op)
    costi=[];
    % --- initial sizes and controls
    n   = size(x0, 1);          % dimension of state vector
    m   = size(u0, 1);          % dimension of control vector
    N   = size(u0, 2);          % number of state transitions
    u   = u0;                   % initial control sequence
    num_obj = n / 4;

    % --- initial trajectory
    [x,cost]  = forward_pass(x0,u,SIMULATE,COST);

    for iter = 1:150
        costi = [costi ,sum(cost)];

        display([int2str(iter) ': ' num2str(sum(cost(:)))]);
        %====== STEP 1: compute Hessian and derivative
        [~,fx,fu,fxx,fxu,fuu] = SIMULATE([u nan(m,1)], 1:N+1);
        [~,cx,cu,cxx,cxu,cuu] = COST([u nan(m,1)], 1:N+1);
        display(['Derivatives: sum(cu):' num2str(sum(abs(cu(:)))) ' sum(cx): ' num2str(sum(abs(cx(:))))]);
        cu = cu(:,1:end-1);
        temp = cellfun(@sparse,  num2cell(fx,[1,2]), 'uni',0);
        fx=blkdiag(temp{1:end-1});
        A = spdiags(ones(size(fx,1)+8*num_obj,1),0,size(fx,1)+4*num_obj,size(fx,1)+4*num_obj);
        A((4*num_obj+1):end,1:(end-4*num_obj))=A((4*num_obj+1):end,1:(end-4*num_obj))-fx;
        fx = A;
        
        temp = cellfun(@sparse,  num2cell(fu,[1,2]), 'uni',0  );
        fu=-blkdiag(temp{1:end-1});
        fu = [zeros(4*num_obj,size(fu,2)); fu];
        S=full(-fx\fu);
        dcdu = S' * cx(:)  + cu(:);
        % verification
        eps = 1e-6;
        du = eps*(rand(size(u))-0.5);
        dx1 = reshape(S*du(:),4,[]);
        [xp,costp]  = forward_pass(x0,u+du,SIMULATE,COST);
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
        H=H+0*sparse(eye(size(H)));

        %====== STEP 2: Line search
        p = reshape(-H\dcdu,2*num_obj,[]);
%         p = reshape(-dcdu,2,[]);
        if(p(:)'*dcdu>0)
            disp('not a search direction');
             s=eig(H'+H);
             if(any(s<0))
                 disp('H not PSD, trying to modify H');
                flag = true;
                mu = 1e-4;
                itr = 0;
                while flag
                    H = H + mu*eye(size(H));
                    s=eig(H'+H);
                    flag = any(s<0);    
                    itr = itr + 1;
                    mu = mu * 2;
                end
                disp(['Modifying H took ' int2str(itr) ' iterations']);
            end
        end
        %TODO: add constant regularization, eigen decomposition is too slow for
        %complex problems, so add h*I to H
%          s=eig(H'+H);
%          if(any(s<0))
%              disp('H not PSD, trying to modify H');
%             flag = true;
%             mu = 1e-4;
%             itr = 0;
%             while flag
%                 H = H + mu*eye(size(H));
%                 s=eig(H'+H);
%                 flag = any(s<0);    
%                 itr = itr + 1;
%                 mu = mu * 2;
%             end
%             disp(['Modifying H took ' int2str(itr) ' iterations']);
%         end
        
        alpha = .2;
        costnew = zeros(size(cost));
        flag = true;
        while flag
            [xnew,costnew] = forward_pass(x0,u+alpha*p,SIMULATE,COST);
            Op.plotFn(x, xnew);
            if sum(cost(:)) < sum(costnew(:))
                alpha = alpha / 2;
                display(['line search fail with new cost ' num2str(sum(costnew(:)))]);
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
end
function [x,cost] = forward_pass(x0,u,SIMULATE,COST)
    n = size(x0,1);
    N = size(u,2);

    x = zeros(n,N);
    cost = zeros(1,N+1);
    x(:,1)=x0;
    
    for i = 1:N
        x(:,i+1) = SIMULATE(u(:,i), i);
    end
    final_u(1:size(u,1)) = nan;
    cost = COST([u final_u']);
    assert(size(cost,1) == 1 & size(cost,2) == N+1);
end