classdef Car < handle
    %CAR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        carObjective
    end
    
    methods
        function obj = Car()
            obj.carObjective = CarObjective;
        end
        
        function h = draw(obj,x,u)
            
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