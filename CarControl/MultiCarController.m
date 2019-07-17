classdef MultiCarController
    properties
        Cars
    end
    
    methods
        function obj = MultiCarController()
            %initialize a cell array Cars with Car objects 
        end
        
        function [c,cx,cu,cxx,cuu,cxu]=cost(obj)
            % compute for each car and then compute all the inter-car costs
        end
        function [f,fx,fu,fxx,fxu,fuu]=simulation(obj)
            
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
        end %This should be moved to an independant file
    end
end