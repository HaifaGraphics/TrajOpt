classdef MultiCarController
    %class for that contains all car objects to be used as a singleton
    %object that can be queried for state updates (eg. by calling
    %dynamics()), costs, and to animate trajectories.
    properties
        Cars
        hypotheticalCar = Car(nan,nan,nan);     %Used by ___Hypothetical functions to perform dynamics and cost calculations on 
                                                %trajectories and control sequences that we don't want to save - 
                                                %currently used to find derivatives of simulate and cost. 
        states_per_car = 4;
        ctrls_per_car = 2;
        xT;                                         %Save global target state and horizon
        T;
    end
    
    methods
        function obj = MultiCarController(num_obj, x0, u0, xT)
            %This constructor initializes a cell array of Car objects
            %with the corresponding x0, u0, and xT
            obj.xT = xT;
            for i=1:num_obj
                obj.Cars{i, 1} = Car(x0((1:obj.states_per_car) + (i-1)*obj.states_per_car),...
                                    u0((1:obj.ctrls_per_car) + (i-1)*obj.ctrls_per_car, :),...
                                    xT((1:obj.states_per_car) + (i-1)*obj.states_per_car));
            end
            obj.T = size(u0,2);
        end
        
        function y = simulate(obj,u,i)
            y = zeros(size(obj.xT));
            for j=0:length(obj.Cars)-1
                u_car = u((1:obj.ctrls_per_car) + j*obj.ctrls_per_car,:);
                y((1:obj.states_per_car) + j*obj.states_per_car,:) = obj.Cars{j+1}.simulate(u_car,i);
            end
        end
        function y = simulateHypothetical(obj,x,u)
            obj.hypotheticalCar.x = x;
            obj.hypotheticalCar.u = u;
            y = obj.hypotheticalCar.simulate(u,1:size(x,2));
            obj.hypotheticalCar.x = nan;
            obj.hypotheticalCar.u = nan;
        end
        function c = cost(obj, u)
            c = 0;
            x = nan(size(obj.xT,1),obj.T+1);
            for i=0:length(obj.Cars)-1
                u_car = u((1:obj.ctrls_per_car) + i*obj.ctrls_per_car,:);
                xT_car = obj.xT((1:obj.states_per_car) + i*obj.states_per_car);
                c = c + obj.Cars{i+1}.cost(u_car);
                x((1:obj.states_per_car) + i*obj.states_per_car,:) = obj.Cars{i+1}.x;
            end

            % Collision soft-constraint
             overlap_length = 0;
             if length(obj.Cars) > 1
                 overlap_length = obj.bbox_overlap(x);
             end
             lc = 1e-1*overlap_length;
            %display(['Overlap Cost: ' num2str(sum(lc(:))) ' Car Cost: ' num2str(sum(c(:)))]);
            c     = c + lc;
        end
        function c = costHypothetical(obj,x,u,xT)
            obj.hypotheticalCar.x = x;
            obj.hypotheticalCar.xT = xT;
            c = obj.hypotheticalCar.cost(u);

            % Collision soft-constraint
             overlap_length = 0;
             if length(obj.Cars) > 1
                 overlap_length = obj.bbox_overlap(x);
             end
             lc = 1e-1*overlap_length;
            
            c     = c + lc;
        end
        function [f,fx,fu,fxx,fxu,fuu] = dynamics(obj,u,i,fullHessian)
            if nargout == 1
                assert(size(u,2) == 1);
                f = obj.simulate(u,i);
            else
                f = [];
%             use helper function finite_difference() to compute derivatives
%                 state and control indices
                num_obj = length(obj.Cars);
                ix = 1:4*num_obj;
                iu = (4*num_obj + 1):(6*num_obj);
                x = [];
                for j=1:num_obj
                    x = [x;obj.Cars{j}.x];
                end
%                 dynamics first derivatives
                xu_dyn  = @(xu) obj.simulateHypothetical(xu(ix,:),xu(iu,:));
                J       = obj.finite_difference(xu_dyn, [x; u]);
                fx      = J(:,ix,:);
                fu      = J(:,iu,:);
%                 dynamics second derivatives
                if fullHessian
                    xu_Jcst = @(xu) obj.finite_difference(xu_dyn, xu);
                    JJ      = obj.finite_difference(xu_Jcst, [x; u]);
                    sJ = size(J);
                    JJ      = reshape(JJ, [4*num_obj 6*num_obj sJ(2:end)]);
                    JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
                    fxx     = JJ(:,ix,ix,:);
                    fxu     = JJ(:,ix,iu,:);
                    fuu     = JJ(:,iu,iu,:);
                else
                    [fxx,fxu,fuu] = deal([]);
                end
            end
        end
        function [c,cx,cu,cxx,cxu,cuu] = costWithDerivatives(obj,u)
            if nargout == 1
                c = obj.cost(u);
            else
                c = [];
%             state and control indices
                num_obj = length(obj.Cars);
                ix = 1:4*num_obj;
                iu = (4*num_obj + 1):(6*num_obj);
                x = [];
                for j=1:num_obj
                    x = [x;obj.Cars{j}.x];
                end
%             use helper function finite_difference() to compute derivatives
%                 cost first derivatives
                xu_cost = @(xu) obj.costHypothetical(xu(ix,:),xu(iu,:),obj.xT);
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
        function overlap_length = bbox_overlap(~,X)
            num_obj = size(X,1) / 4;
            center_X = X(1:4:end,:) + cos(X(3:4:end,:))*1.1;
            center_Y = X(2:4:end,:) + sin(X(3:4:end,:))*1.1;
            overlap_length = 0;
            for i=1:num_obj
                for j=i+1:num_obj
                    c1 = [center_X(i,:); center_Y(i,:)];
                    c2 = [center_X(j,:); center_Y(j,:)];
                    curr_overlap = max(4.6 - sqrt((c1(1,:)-c2(1,:)).^2 + (c1(2,:)-c2(2,:)).^2), 0).^2;
                    overlap_length = overlap_length + curr_overlap;
                end
            end
        end
        function animateTrajectories(obj)
            figure(9)
            for t=1:obj.T
               handles = [];
               set(0,'currentfigure',9);
               for j=1:length(obj.Cars)
                   car = obj.Cars{j};
                   handles = [handles; car.drawAtTimestep(t)];
               end
               drawnow
               for k=1:length(obj.Cars)
                   delete(handles(k,:))
               end
            end
        end        
    end
end