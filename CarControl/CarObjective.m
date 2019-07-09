classdef CarObjective
%     CAROBJECTIVE Summary of this class goes here
%       Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = CarObjective()
        end
        
        function y = dynamics(~,x,u)
            
%             === states and controls:
%             x = [x y t v]' = [x; y; car_angle; front_wheel_velocity]
%             u = [w a]'     = [front_wheel_angle; acceleration]
            
%             constants
            d  = 2.0;      % d = distance between back and front axles
            h  = 0.03;     % h = timestep (seconds)
            
%             controls
            w  = u(1,:,:); % w = front wheel angle
            a  = u(2,:,:); % a = front wheel acceleration
            
            o  = x(3,:,:); % o = car angle
%             z = unit_vector(o)
            z  = [cos(o); sin(o)];
            
            v  = x(4,:,:); % v = front wheel velocity
            f  = h*v;      % f = front wheel rolling distance
%             b = back wheel rolling distance
            b  = d + f.*cos(w) - sqrt(d^2 - (f.*sin(w)).^2);
            if ~isreal(b)
                error('FOUND IMG b')
            end
%             do = change in car angle
            do = asin(sin(w).*f/d);
            
            dy = [[b;b].*z; do; h*a];   % change in state
            y  = x + dy;                % new state
        end
        function c = cost(~,x, u)
%             cost function for car-parking problem
%             sum of 3 terms:
%             lu: quadratic cost on controls
%             lf: final cost on distance from target parking configuration
%             lx: running cost on distance from origin to encourage tight turns
            
            final = isnan(u(1,:));
            u(:,final)  = 0;
            
            cu  = 1e-2*[1 .25];         % control cost coefficients
            
            cf  = [ .1  .1   1  .3];    % final cost coefficients
            pf  = [.01 .01 .01  1]';    % smoothness scales for final cost
            
            cx  = 1e-3*[1  1];          % running cost coefficients
            
%             control cost
            lu    = cu*u.^2;
            angle_lim = 0.4;
            %a = (10*(abs(u(1,:))-angle_lim)).^2;
            a = (abs(u(1,:))-angle_lim).^2;
            a(abs(u(1,:))<angle_lim) = 0;
            
            acc_lim = 1.8;
            %b = (10*(abs(u(2,:))-acc_lim)).^2;
            b = (abs(u(2,:))-acc_lim).^2;
            b(abs(u(2,:))<acc_lim) = 0;
            
%             final cost
            if any(final)
                llf      = cf*x(:,final).^2;
                lf       = double(final);
                lf(final)= llf;
            else
                lf    = 0;
            end
%             running cost
            lx = cx*x(1:2,:).^2;
            
            c     = lu + 10^3*a + 10^3*b + lx + 100*lf;
        end
        
        function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = dyn_cst(obj,x,u,fullHessian)
%             combine car dynamics and cost
%             use helper function finite_difference() to compute derivatives
            
            if nargout == 2
                f = obj.dynamics(x,u);
                c = obj.cost(x,u);
            else
%                 state and control indices
                ix = 1:4;
                iu = 5:6;
                
%                 dynamics first derivatives
                xu_dyn  = @(xu) obj.dynamics(xu(ix,:),xu(iu,:));
                J       = obj.finite_difference(xu_dyn, [x; u]);
                fx      = J(:,ix,:);
                fu      = J(:,iu,:);
                
%                 dynamics second derivatives
                if fullHessian
                    xu_Jcst = @(xu) obj.finite_difference(xu_dyn, xu);
                    JJ      = obj.finite_difference(xu_Jcst, [x; u]);
                    sJ = size(J);
                    JJ      = reshape(JJ, [4 6 sJ(2:end)]);
                    JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
                    fxx     = JJ(:,ix,ix,:);
                    fxu     = JJ(:,ix,iu,:);
                    fuu     = JJ(:,iu,iu,:);
                else
                    [fxx,fxu,fuu] = deal([]);
                end
                
%                 cost first derivatives
                xu_cost = @(xu) obj.cost(xu(ix,:),xu(iu,:));
                J       = squeeze(obj.finite_difference(xu_cost, [x; u]));
                cx      = J(ix,:);
                cu      = J(iu,:);
                
%                 cost second derivatives
                xu_Jcst = @(xu) squeeze(obj.finite_difference(xu_cost, xu));
                JJ      = obj.finite_difference(xu_Jcst, [x; u]);
                JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
                cxx     = JJ(ix,ix,:);
                cxu     = JJ(ix,iu,:);
                cuu     = JJ(iu,iu,:);
                
                [f,c] = deal([]);
            end
        end
        function J = finite_difference(~,fun, x, h)
%             simple finite-difference derivatives
%             assumes the function fun() is vectorized
            
            if nargin < 4
                h = 2^-17;
            end
            
            function c = pp(a,b)
                c = bsxfun(@plus,a,b);
            end
            [n, K]  = size(x);
            H       = [zeros(n,1) h*eye(n)];
            H       = permute(H, [1 3 2]);
            X       = pp(x, H);
            X       = reshape(X, n, K*(n+1));
            Y       = fun(X);
            m       = numel(Y)/(K*(n+1));
            Y       = reshape(Y, m, K, n+1);
            J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
            J       = permute(J, [1 3 2]);
            
        end
    

    end
end