classdef Car < handle
%class for drawing and representing the state of a car.
% The trajectory and control at time i are updated when simulate(u, i) is
% called.

    properties
        x % trajectory
        u % control
        xT % Target state
        angleLim;
        accLim;
    end
    
    methods
        function obj = Car(x0, u0, xT, lims)
            obj.x = nan(size(x0,1),size(u0,2));
            obj.x(:,1) = x0;
            obj.u = u0;
            obj.xT = xT;
            obj.angleLim = lims(1);
            obj.accLim = lims(2);
        end
        
        function c = cost(obj,u)
%             cost function for car-parking problem
%             sum of 3 terms:
%             lu: quadratic cost on controls
%             lf: final cost on distance from target parking configuration
%             lx: running cost on distance from target parking location to encourage tight turns

            final = isnan(u(1,:));
            u(:,final)  = 0;
            
            x = obj.x;
            xT = obj.xT;
            
            num_obj = size(x,1) / 4;
            
            %TODO: increase to 10^4
            cc  = repmat(1e1,1,num_obj);    % control soft-constraint coefficients
            cu  = 1e-2*[1 .25];             % control cost coefficients
            
            cf  = [ .1  .1   1  .3];    % final cost coefficients
            %pf  = [.01 .01 .01  1]';    % smoothness scales for final cost
            
            cx  = 1e-3*[1  1 0 0];          % running cost coefficients
            
%           pad coefficients for multiple cars
            cu = repmat(cu,1,num_obj);
            cf = repmat(cf,1,num_obj);
            cx = repmat(cx,1,num_obj);
            
%           control cost
            lu    = cu*u.^2;
            a = (abs(u(1:2:end,:))-obj.angleLim).^2;
            a(abs(u(1:2:end,:))<obj.angleLim) = 0;
            lang = cc*a;
            
            b = (abs(u(2:2:end,:))-obj.accLim).^2;
            b(abs(u(2:2:end,:))<obj.accLim) = 0;
            lacc = cc*b;

%             final cost
            if any(final)
                llf      = cf*(x(:,final)-xT).^2;
                lf       = double(final);
                lf(final)= llf;
            else
                lf    = 0;
            end
%             running cost
            lx = cx*(x-obj.xT).^2;
            
            c     = lu + lang + lacc + lx + 100*lf;
            display(['Control Cost: ' num2str(sum(lu(:))) ' Limit Costs: ' num2str(sum(lang(:)) + sum(lacc(:))) ' Running Cost: ' num2str(sum(lx(:))) ' Final Cost: ' num2str(sum(100*lf(:)))] );
        end
        function y=simulate(obj,u,i)
%             === states and controls:
%             x = [x1 y1 t1 v1]' = [x; y; car_angle; front_wheel_velocity]
%             u = [w1 a1]'     = [front_wheel_angle; acceleration]

            x = obj.x(:,i);
%             constants
            d  = 2.0;      % d = distance between back and front axles
            h  = 0.03;     % h = timestep (seconds)

            %             controls
            w  = u(1:2:end,:,:); % w = front wheel angle
            a  = u(2:2:end,:,:); % a = front wheel acceleration



            v  = x(4:4:end,:,:); % v = front wheel velocity
            f  = h*v;      % f = front wheel rolling distance
%             b = back wheel rolling distance
            b  = d + f.*cos(w) - sqrt(d^2 - (f.*sin(w)).^2);
            
            % if f is large enough, we can have complex values for b and d
            % that break the small-angle approximation used here.
            b(~isreal(b)) = nan;
%             do = change in car angle
            do = asin(sin(w).*f/d);
            do(~isreal(do)) = nan;
            
            if any(isnan(do)) | any(isnan(b))
                disp("Invalid simulation step found, assigning infinite cost");
            end
            dy = [];
            for j=1:size(x,1)/4
                o  = x(3 + 4*(j-1),:,:); % o = car angle
    %             z = unit_vector(o)
                z  = [cos(o); sin(o)];
                dy = [dy; [b(j,:);b(j,:)].*z; do(j,:); h*a(j,:)]; % change in state
            end          
            y = x + dy;                         % new state
            %Update internal trajectory and control
            obj.x(:,i+1) = y;
            obj.u(:,i) = u;
        end
        function h = drawAtTimestep(obj,t)
            h = obj.draw(obj.x(:,t),obj.u(:,t),true);
        end
        function h = draw(obj,x,u,draw_bbox)
            
            body        = [0.9 2.1 0.3];           % body = [width length curvature]
            bodycolor   = 0.5*[1 1 1];
            headlights  = [0.25 0.1 .1 body(1)/2]; % headlights [width length curvature x]
            lightcolor  = [1 1 0];
            wheel       = [0.15 0.4 .06 1.1*body(1) -1.1 .9];  % wheels = [width length curvature x yb yf]
            wheelcolor  = 'k';
            
            h = [];
            
            % make wheels
            for front = 1:2
                for right = [-1 1]
                    h(end+1) = obj.rrect(wheel,wheelcolor)'; %#ok<AGROW>
                    if front == 2
                        obj.twist(h(end),0,0,u(1))
                    end
                    obj.twist(h(end),right*wheel(4),wheel(4+front))
                end
            end
            
            % make body
            h(end+1) = obj.rrect(body,bodycolor);
            
            % make window (hard coded)
            h(end+1) = patch([-.8 .8 .7 -.7],.6+.3*[1 1 -1 -1],'w');
            
            % headlights
            h(end+1) = obj.rrect(headlights(1:3),lightcolor);
            obj.twist(h(end),headlights(4),body(2)-headlights(2))
            h(end+1) = obj.rrect(headlights(1:3),lightcolor);
            obj.twist(h(end),-headlights(4),body(2)-headlights(2))
            
            % put rear wheels at (0,0)
            obj.twist(h,0,-wheel(5))
            
            % align to x-axis
            obj.twist(h,0,0,-pi/2)
            
            % make origin (hard coded)
            ol = 0.1;
            ow = 0.01;
            h(end+1) = patch(ol*[-1 1 1 -1],ow*[1 1 -1 -1],'k');
            h(end+1) = patch(ow*[1 1 -1 -1],ol*[-1 1 1 -1],'k');
            
            obj.twist(h,x(1),x(2),x(3))
            
            % Draw bounding box
            if draw_bbox
                center = [x(1) + cos(x(3))*1.1, x(2) + sin(x(3))*1.1];
                h(end+1) = viscircles(center,2.3,'LineWidth',0.5);
            end
        end
        function twist(~,handle,x,y,theta)
            % a planar twist: rotate object by theta, then translate by (x,y)
            i = 1i;
            if nargin == 4
                theta = 0;
            end
            for h = handle
                Z = get(h,'xdata') + i*get(h,'ydata');
                Z = Z * exp(i*theta);
                Z = Z + (x + i*y);
                set(h,'xdata',real(Z),'ydata',imag(Z));
            end
            
        end
        function h = rrect(~,wlc, color)
            % draw a rounded rectangle (using complex numbers and a kronecker sum :-)
            
            N        = 25; % number of points per corner
            
            width    = wlc(1);
            length   = wlc(2);
            curve    = wlc(3);
            
            a        = linspace(0,2*pi,4*N);
            circle   = curve*exp(1i*a);
            width    = width-curve;
            length   = length-curve;
            rect1    = diag(width*[1 -1 -1 1] + 1i*length *[1 1 -1 -1]);
            rectN    = sum(kron(rect1, ones(1,N)), 1) ;
            rr       = circle + rectN;
            rr       = [rr rr(1)]; % close the curve
            
            h        = patch(real(rr),imag(rr),color);
        end
    end
end