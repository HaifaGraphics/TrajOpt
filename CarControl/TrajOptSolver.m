classdef TrajOptSolver < matlab.mixin.Copyable
    properties
        ForceStop
        Stop

        num_obj;                    % number of cars
        T;                          % horizon
        lims  = [-.5 .5;            % wheel angle limits (radians) - must be symmetric about 0
                 -4.5  4.5];        % acceleration limits (m/s^2)
        x0;                         % initial state
        u0;                         % initial controls
        xT;                         % target state
        costN;                      % total costs and iter count updated after every iteration
        costDDP;
        iter = 0;
        
        controllerN;                % these objects track and manage the optimization problems and relevant drawing functions
        controllerDDP;

        % Set full_DDP=true to compute 2nd order derivatives of the 
        % dynamics. This will make iterations more expensive, but 
        % final convergence will be much faster (quadratic)
        full_DDP = false;

        SIMULATE_N;
        COST_N;
        SIMULATE_DDP;
        COST_DDP;
        doNewton = true;
        doDDP    = true;
        isRand;
        
        line_handles; %l_h(1,:) returns trajectory lines for every car for Newton, l_h(2,:) does the same for DDP (e.g l_h(2,1) for DDP Car 1).
        hyp_line_handles;
        targetCar_handles;
        plotFn = @plot_trajectory;
        Op;
        
        xN;
        uN;
        xDDP;
        uDDP;
        
        initBtn;    %we need handles to these guui buttons since they can interrupt the solver loop
        randBtn;
    end
    properties(SetAccess = private)

    end
    methods
        function obj=NewtonSolver

        end
        function Initialize(obj,x0,xT,MainAxes,isRand)
            obj.num_obj = 2;                                % number of cars
            obj.T       = 250;                              % horizon
            obj.lims  = [-.5 .5;                            % wheel angle limits (radians) - must be symmetric about 0
                         -4.5  4.5];                        % acceleration limits (m/s^2)
            obj.x0      = reshape(x0',[],1);      % initial state
            obj.xT      = reshape(xT',[],1);        % target state
            obj.SIMULATE_N  = @(u,i) obj.controllerN.dynamics(u,i,obj.full_DDP);
            obj.COST_N      = @(u,i) obj.controllerN.costWithDerivatives(u);
            obj.SIMULATE_DDP  = @(u,i) obj.controllerDDP.dynamics(u,i,obj.full_DDP);
            obj.COST_DDP      = @(u,i) obj.controllerDDP.costWithDerivatives(u);
            
            delete(obj.line_handles);
            delete(obj.hyp_line_handles);
            delete(obj.targetCar_handles);
            obj.targetCar_handles = [];
            line_handles = gobjects(2, obj.num_obj);
            hyp_line_handles = gobjects(2, obj.num_obj);
            
            for i=1:2
               % plot target configuration with light colors
                axes(MainAxes(i));
                handles = [];
                car = Car([0 0 0 0]',nan,nan,obj.lims(:,2));
                for j=1:obj.num_obj
                    handles = [handles; car.draw(obj.xT((1:4)+4*(j-1)), [0 0]', false)];
                end
                fcolor  = get(handles,'facecolor');
                ecolor  = get(handles,'edgecolor');
                fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
                ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
                set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])
                
                colorstring = 'brgky';
                for k=1:obj.num_obj
                    obj.line_handles(i,k) = line([0 0],[0 0],'color',colorstring(k),'linewidth',2);
                    obj.hyp_line_handles(i,k) = line([0 0],[0 0],'color',colorstring(k),'LineStyle','--','linewidth',1);
                end
                obj.targetCar_handles = [obj.targetCar_handles; handles];
            end
        
            obj.Op.lims = obj.lims;
            obj.Op.plotFn = obj.plotFn;
            obj.Stop=0;
            obj.iter = 0;
            
            if ~isRand
                obj.u0      = repmat(obj.lims(:,1),obj.num_obj,obj.T) + repmat(obj.lims(:,2) - obj.lims(:,1),obj.num_obj,obj.T) * 0.6; % initial controls
                obj.isRand = false;
            else
                obj.u0      = repmat(obj.Op.lims(:,1),obj.num_obj,obj.T) + repmat(obj.Op.lims(:,2) - obj.Op.lims(:,1),obj.num_obj,obj.T) .* rand(2*obj.num_obj, obj.T); % initial controls
                obj.isRand = true;
            end
            
            obj.controllerN = MultiCarController(obj.num_obj, obj.x0, obj.u0, obj.xT, obj.lims);
            obj.controllerDDP = MultiCarController(obj.num_obj, obj.x0, obj.u0, obj.xT, obj.lims);
            
            obj.xN = obj.x0;
            obj.uN = obj.u0;
            obj.xDDP = obj.x0;
            obj.uDDP = obj.u0;
            obj.costN = 0;
            obj.costDDP = 0;
            
            notify(obj,'IterationDone');    %callback updates gui iter and cost values
        end

        function StartInteractive(obj,MainAxes)
            disp('started')
            obj.Stop = 0;
            while ~obj.Stop
                % === run the optimization!
                if obj.doNewton
                    axes(MainAxes(1));
                    obj.Op.plotFn = @(x)plot_trajectory(obj.line_handles(1,:), obj.hyp_line_handles(1,:), x); %assign the right plot and handles
                    [obj.xN,obj.uN,costN,costNi]= Newton(obj.SIMULATE_N, obj.COST_N, obj.x0, obj.uN, obj.Op,true);
                    obj.costN = sum(costN);
                end
                if obj.doDDP
                    axes(MainAxes(2));
                    obj.Op.plotFn = @(x)plot_trajectory(obj.line_handles(2,:), obj.hyp_line_handles(2,:), x);
                    [obj.xDDP,obj.uDDP,~,~,~,costDDP,costDDPi]= DDP(obj.SIMULATE_DDP, obj.COST_DDP, obj.x0, obj.uDDP, obj.Op,true); %assign the right plot and handles
                    obj.costDDP = sum(costDDP);
                end
                
                obj.iter = obj.iter + 1;
                notify(obj,'IterationDone');    %callback updates gui iter and cost values
                obj.Stop = obj.Stop | obj.iter == 200 | ~(obj.isRand & ~obj.initBtn.Value() & obj.randBtn.Value()) & ~(~obj.isRand & ~obj.randBtn.Value() & obj.initBtn.Value());
            end
            if obj.iter == 150              %Optimization Done!
                obj.checkAndAnimate(MainAxes);
                obj.StopInteractive();
            end
            disp('stopped')
        end
        function checkAndAnimate(obj, MainAxes)
            if obj.doNewton & ~obj.controllerN.check_legality()
                display(obj.uN);
                display('Illegal Control Sequence - Newton');
            else
                %Disp max control utilization% metric to ensure we aren't restricting
                %ourselves from using all of the available control bandwidth
                display('Max Control Utilization% - Newton');
                display((max(obj.uN,[],2) - min(obj.uN,[],2)) ./ repmat((obj.Op.lims(:,2) - obj.Op.lims(:,1)),obj.num_obj,1) .* 100);
            end
            if obj.doDDP & ~obj.controllerDDP.check_legality()
                display(obj.uDDP);
                display('Illegal Control Sequence - DDP');
            else
                %Disp max control utilization% metric to ensure we aren't restricting
                %ourselves from using all of the available control bandwidth
                display('Max Control Utilization% - DDP');
                display((max(obj.uDDP,[],2) - min(obj.uDDP,[],2)) ./ repmat((obj.Op.lims(:,2) - obj.Op.lims(:,1)),obj.num_obj,1) .* 100);
            end
            % animate the resulting trajectories
            if obj.doNewton
                axes(MainAxes(1));
                obj.controllerN.animateTrajectories();
            end
            if obj.doDDP   
                axes(MainAxes(2));
                obj.controllerDDP.animateTrajectories();
            end
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
        function setButtons(obj, initBtn, randBtn)
            obj.initBtn = initBtn;
            obj.randBtn = randBtn;
        end
        function enableNewton(obj,enable)
            obj.doNewton = enable;
        end
        function enableDDP(obj,enable)
            obj.doDDP = enable;
        end
        function isEnabled = NewtonEnabled(obj)
            isEnabled = obj.doNewton;
        end
        function isEnabled = DDPEnabled(obj)
            isEnabled = obj.doDDP;
        end
        function rand = isRandom(obj)
            rand = obj.isRand;
        end
        function it = getIter(obj)
            it = obj.iter;
        end
        function c = getNewtonCost(obj)
            c = obj.costN;
        end
        function c = getDDPCost(obj)
            c = obj.costDDP;
        end
    end
    events
        IterationDone
    end
end
% prepare and install trajectory visualization callback
function plot_trajectory(line_handles,hyp_line_handles,x)
    num_obj = length(line_handles);
    for i=1:num_obj
        set(line_handles(i),'Xdata',x(1+(i-1)*4,:),'Ydata',x(2+(i-1)*4,:));
        if size(x,1) > num_obj*4
            set(hyp_line_handles(i),'Xdata',x(1+(num_obj + i - 1)*4,:),'Ydata',x(2+(num_obj + i - 1)*4,:));
            pause(0.01);
        end
    end
end

