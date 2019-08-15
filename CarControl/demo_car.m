clc;
close all

fprintf(['\nA demonstration of the iLQG algorithm '...
'with car parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% set up the optimization problem
global num_obj;
num_obj = 2;                % number of cars
T       = 250;              % horizon
Op.lims  = [-.5 .5;         % wheel angle limits (radians) - must be symmetric about 0
             -4.5  4.5];        % acceleration limits (m/s^2)
x0      = [-5;-5;pi/4;0;-5;5;-pi/4;0];%;0;-5;5;0;0];   % initial state
u0      = repmat(Op.lims(:,1),num_obj,T) + repmat(Op.lims(:,2) - Op.lims(:,1),num_obj,T) * 0.6; % initial controls
xT      = [5;5;pi/4;0;5;-5;-pi/4;0]; % target state

controllerN = MultiCarController(num_obj, x0, u0, xT, Op.lims);
controllerDDP = MultiCarController(num_obj, x0, u0, xT, Op.lims);

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

SIMULATE_N  = @(u,i) controllerN.dynamics(u,i,full_DDP);
COST_N      = @(u,i) controllerN.costWithDerivatives(u);
SIMULATE_DDP  = @(u,i) controllerDDP.dynamics(u,i,full_DDP);
COST_DDP      = @(u,i) controllerDDP.costWithDerivatives(u);
doNewton = true;
doDDP    = true;

% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','car parking','Menu','none','NumberT','off')
subplot(1,2,1);

axesTitles = ["Newton","DDP"];
for i=1:2
   axes(i) = subplot(1,2,i);
   set(axes(i),'xlim',[-10 10],'ylim',[-10 10],'DataAspectRatio',[1 1 1])
   title(axesTitles(i));
   grid on
   box on
   
   % plot target configuration with light colors
    handles = [];
    car = Car([0 0 0 0]',nan,nan,Op.lims(:,2));
    for i=1:num_obj
        handles = [handles; car.draw(xT((1:4)+4*(i-1)), [0 0]', false)];
    end
    fcolor  = get(handles,'facecolor');
    ecolor  = get(handles,'edgecolor');
    fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
    ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
    set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])
end

plotFn = @plot_trajectory;
Op.plotFn = plotFn;

% === run the optimization!

if doNewton
    prepare_subplot(1, num_obj);
    [xN,uN,costN,costNi]= Newton(SIMULATE_N, COST_N, x0, u0, Op);
    if ~controllerN.check_legality()
        display(uN);
        display('Illegal Control Sequence - Newton');
    else
        %Disp max control utilization% metric to ensure we aren't restricting
        %ourselves from using all of the available control bandwidth
        display('Max Control Utilization% - Newton');
        display((max(uN,[],2) - min(uN,[],2)) ./ repmat((Op.lims(:,2) - Op.lims(:,1)),num_obj,1) .* 100);
    end
end
if doDDP
    prepare_subplot(2, num_obj);
    [xDDP,uDDP,~,~,~,costDDP,costDDPi]= DDP(SIMULATE_DDP, COST_DDP, x0, u0, Op);
    if ~controllerDDP.check_legality()
        display(uDDP);
        display('Illegal Control Sequence - DDP');
    else
        %Disp max control utilization% metric to ensure we aren't restricting
        %ourselves from using all of the available control bandwidth
        display('Max Control Utilization% - DDP');
        display((max(uDDP,[],2) - min(uDDP,[],2)) ./ repmat((Op.lims(:,2) - Op.lims(:,1)),num_obj,1) .* 100);
    end
end

% animate the resulting trajectories
if doNewton
    subplot(1,2,1);
    controllerN.animateTrajectories();
end
if doDDP   
    subplot(1,2,2);
    controllerDDP.animateTrajectories();
end

function plot_trajectory(xIn)
    global num_obj;
    x = xIn(1:num_obj*4,:);
    xhyp = xIn((num_obj*4+1):end,:);
    global line_handles;
    global hyp_line_handles;
    for i=1:num_obj
        set(line_handles(i),'Xdata',x(1+(i-1)*4,:),'Ydata',x(2+(i-1)*4,:));
        if nargin > 1
            set(hyp_line_handles(i),'Xdata',xhyp(1+(i-1)*4,:),'Ydata',xhyp(2+(i-1)*4,:));
            pause(0.01);
        end
    end
end

% prepare and install trajectory visualization callback
function prepare_subplot(plot, num_obj)
    subplot(1,2,plot);
    global line_handles;
    global hyp_line_handles;
    line_handles = gobjects(num_obj);
    hyp_line_handles = gobjects(num_obj);
    colorstring = 'brgky';
    for i=1:num_obj
        line_handles(i) = line([0 0],[0 0],'color',colorstring(i),'linewidth',2);
        hyp_line_handles(i) = line([0 0],[0 0],'color',colorstring(i),'LineStyle','--','linewidth',1);
    end
end