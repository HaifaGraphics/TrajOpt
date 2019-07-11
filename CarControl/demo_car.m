clc;
close all

fprintf(['\nA demonstration of the iLQG algorithm '...
'with car parking dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])


car = Car;
carobj = car.carObjective;
% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(x,u,i) carobj.dyn_cst(x,u,full_DDP);
T       = 300;              % horizon
num_obj = 2;                % number of cars
x0      = [2;2;pi*3/2;0;-2;-2;-pi*3/2;0];%;0;-5;5;0;0];   % initial state
u0      = rand(2*num_obj,T);    % initial controls
Op.lims  = [-.5 .5;         % wheel angle limits (radians)
             -2  2];        % acceleration limits (m/s^2)

% prepare the visualization window and graphics callback
figure(9);
set(gcf,'name','car parking','Menu','none','NumberT','off')
set(gca,'xlim',[-10 10],'ylim',[-10 10],'DataAspectRatio',[1 1 1])
grid on
box on

% plot target configuration with light colors
handles = car.draw([0 0 0 0]', [0 0]');
fcolor  = get(handles,'facecolor');
ecolor  = get(handles,'edgecolor');
fcolor  = cellfun(@(x) (x+3)/4,fcolor,'UniformOutput',false);
ecolor  = cellfun(@(x) (x+3)/4,ecolor,'UniformOutput',false);
set(handles, {'facecolor','edgecolor'}, [fcolor ecolor])

% prepare and install trajectory visualization callback
global line_handles;
line_handles = gobjects(num_obj);
colorstring = 'brgky';
for i=1:num_obj
    line_handles(i) = line([0 0],[0 0],'color',colorstring(i),'linewidth',2);
end
%line_handle = line([0 0],[0 0],'color','b','linewidth',2);
plotFn = @plot_trajectory;
Op.plotFn = plotFn;

% === run the optimization!
[xN,uN,costN,costNi]= Newton(DYNCST, x0, u0, Op);
%[xDDP,uDDP,~,~,~,costDDP,costDDPi]= DDP(DYNCST, x0, u0, Op);
% check control sequence legality
if ~isempty(Op.lims)
    m   = size(u0, 1) / num_obj;
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
    lim_min = repmat(Op.lims(:,1),num_obj,T);
    lim_max = repmat(Op.lims(:,2),num_obj,T);
    if ~all(uN > lim_min & uN < lim_max, 'all')
        display(uN);
        display('Illegal control sequence - Newton')
    elseif ~all(uDDP > lim_min & uDDP < lim_max, 'all')
        display(uDDP);
        display('Illegal control sequence - DDP')
    else
        %Disp max control utilization% metric to ensure we aren't restricting
        %ourselves from using all of the available control bandwidth
        display('Max Control Utilization% - Newton, Car 1');
        display((max(uN,[],2) - min(uN,[],2)) ./ repmat((Op.lims(:,2) - Op.lims(:,1)),num_obj,1) .* 100);
    end
end
% animate the resulting trajectory
figure(9)
for i=1:T
   handles = [];
   set(0,'currentfigure',9);
   for j=0:num_obj-1
       handles = [handles; car.draw(xN((1:4) + 4*j,i), uN((1:2) + 2*j,i))];
   end
   drawnow
   for k=1:num_obj
    delete(handles(k,:))
   end
end

function plot_trajectory(x)
    num_obj = size(x,1) / 4;
    global line_handles;
    for i=1:num_obj
        set(line_handles(i),'Xdata',x(1+(i-1)*4,:),'Ydata',x(2+(i-1)*4,:));
    end
end