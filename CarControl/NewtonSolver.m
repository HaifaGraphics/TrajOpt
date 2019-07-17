classdef NewtonSolver < matlab.mixin.Copyable
    properties
        ForceStop
        Stop
    end
    properties(SetAccess = private)

    end
    methods
        function obj=NewtonSolver

        end

        function StartInteractive(obj)
            % initialize
            obj.Stop=0; 
            disp('started')
            i=0;
            while ~obj.Stop
                if mod(i,1)==0
                    notify(obj,'IterationDone');
                end
            end
            disp('stopped')
        end
        function StopInteractive(obj)
            obj.Stop=1;
        end
        
        function x = Check(obj)
            %%whatever you want to check and send to the base workspace
            %assignin('base', 'x', x);
        end
        function stop = OnIteration(obj,varargin)
            persistent iter
            if isempty(iter)
                iter=1;
            end
            if obj.ForceStop
                stop = true;
                obj.ForceStop=false;
            else
                stop = false;
            end
            if iter==1
                notify(obj,'IterationDone');
                iter=1;
            else
                iter=iter+1;
            end
        end
    end
    events
        IterationDone
    end
end

