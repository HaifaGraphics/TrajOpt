function TrajOptGui
close all;
myGui = gui.autogui('Location','float');
myGui.PanelWidth=400;
set(gcf,'Renderer','OpenGL');

% Always on top
% warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
% jFrame = get(handle(myGui.UiHandle),'JavaFrame'); drawnow;
% jFrame_fHGxClient = jFrame.fHG2Client;
% jFrame_fHGxClient.getWindow.setAlwaysOnTop(true);

fh = figure('WindowKeyPressFcn',@OnKeyPress);
subplot(1,2,1);

axesTitles = ["Newton","DDP"];
for i=1:2
   MainAxes(i) = subplot(1,2,i);
   set(MainAxes(i),'xlim',[-10 10],'ylim',[-10 10],'DataAspectRatio',[1 1 1],'ButtonDownFcn',@OnLeftAxesDown)
   %axis equal;
   title(axesTitles(i));
   grid on
   box on
end

%% Global variables
axes(MainAxes);

hold on
clickflag = false;

Solver=TrajOptSolver;
hLis=addlistener(Solver,'IterationDone',@OnSolverIter);

active=false;
%% GUI initialization
BtnInitialize = gui.togglebutton('Initialize');
BtnInitialize.ValueChangedFcn = @OnInitialize;

BtnRandomize = gui.togglebutton('Randomize');
BtnRandomize.ValueChangedFcn = @OnRandomize;

TxtMenuEnergyType = gui.textmenu('Method',{'Newton and DDP','Newton','DDP'});
TxtMenuEnergyType.Value='Newton and DDP';
TxtMenuEnergyType.ValueChangedFcn = @OnUpdateParams;

sliderTimeStep = gui.intslider('Time step', [0 40]);
sliderTimeStep.ValueChangedFcn = @onSliderEigenNum;

LabelParams = gui.label('');
Label1Status = gui.label('');
Label2Status = gui.label('');
Label1Status.Position.width = 400;
Label2Status.Position.width = 400;

BtnLoadSource = gui.pushbutton('Check something');
BtnLoadSource.ValueChangedFcn = @OnButtonCheckSomething;

%Since the solver contains a loop, it needs to be able to be able to check
%and exit appropriately when the toggle buttons change since the callbacks
%below won't be triggered. We could have everything run through callbacks
%here but then the complicated solver code would have to go here.
Solver.setButtons(BtnInitialize, BtnRandomize);
settingsUpToDate = false;

%% Callbacks
    function OnInitialize(hObject)
        if hObject.Value()
            %Only one of Initialize and Randomize may be selected
            if BtnRandomize.Value()
                BtnRandomize.Value = 0;
                Solver.StopInteractive();
            end
            if ~settingsUpToDate
                updateSettings(TxtMenuEnergyType.Value)
            end
            Solver.StartInteractive(MainAxes);
        else
            Solver.StopInteractive();
        end
        %We might be exiting because Randomize was selected, manually
        %trigger the missed callback
        if BtnRandomize.Value()
            OnRandomize(BtnRandomize);
        end
    end
    function OnRandomize(hObject)
        if hObject.Value()
            %Only one of Initialize and Randomize may be selected
            if BtnInitialize.Value()
                BtnInitialize.Value = 0;
                Solver.StopInteractive();
            end
            if ~settingsUpToDate
                updateSettings(TxtMenuEnergyType.Value)
            end
            Solver.StartInteractive(MainAxes, true);  
        else
            Solver.StopInteractive();
        end
        %We might be exiting because Initialize was selected, manually
        %trigger the missed callback
        if BtnInitialize.Value()
            OnInitialize(BtnInitialize);
        end
    end
    function OnUpdateParams(~)
        settingsUpToDate = false;
    end

    function OnLeftAxesDown(src,evtdata)
        modifier = get(gcf,'SelectionType');
        pos=get(gca,'currentpoint'); pos=pos(1,1:2)';
        switch modifier
            case 'alt'  % in windows this is 'control'
            otherwise
                clickflag = true;
            return;
        end
    end
    function OnKeyPress(src,evtdata)
        switch evtdata.Character
            case 's'
                if active==false
                    active=true;
                    Parameterizer.StartInteractiveDeform;
                else
                    active=false;
                    Parameterizer.StopInteractiveDeform;
                end
            case 'i'
                OnInitialize;
            case 'r'
                OnRandomize
        end
        OnUpdateParams;
    end
    function OnSolverIter(src,evtdata)
        Label1Status.Value = {['Iter: ',num2str(Solver.getIter())]};
        Label1Status.Position.width = 400;  %This shouldn't be necessary
        label2 = {};
        if Solver.NewtonEnabled()
            label2 = {['Newton Cost: ', num2str(Solver.getNewtonCost())]};
                                 %['Gradient norm :', num2str(7)]};
        end
        if Solver.DDPEnabled()
            if ~isempty(label2)
                label2 = {label2{1};
                    ['DDP Cost: ', num2str(Solver.getDDPCost())]};
            else
                label2 = {['DDP Cost: ', num2str(Solver.getDDPCost())]};
            end
        end
        Label2Status.Value = label2;
        Label2Status.Position.height = 70;  %This shouldn't be necessary
        Label2Status.Position.width = 400;  %This shouldn't be necessary
        Redraw;
    end
    function OnButtonCheckSomething(src,evtdata)
        Redraw;
    end
%% Update solver settings
    function updateSettings(method)
        if strcmp(method,'Newton and DDP')
            Solver.enableNewton(true);
        elseif strcmp(method,'Newton')
            Solver.enableNewton(true);
            Solver.enableDDP(false);
        elseif strcmp(method,'DDP')
            Solver.enableNewton(false);
            Solver.enableDDP(true);
        end    
    end
         
%% Drawing functions
    function Redraw(~)
        % draw cars
        %drawnow;
    end
end