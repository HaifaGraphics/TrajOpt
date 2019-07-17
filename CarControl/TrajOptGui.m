function TrajOptGui
close all;
myGui = gui.autogui('Location','float');
myGui.PanelWidth=200;
set(gcf,'Renderer','OpenGL');

% Always on top
% warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
% jFrame = get(handle(myGui.UiHandle),'JavaFrame'); drawnow;
% jFrame_fHGxClient = jFrame.fHG2Client;
% jFrame_fHGxClient.getWindow.setAlwaysOnTop(true);

fh = figure('WindowKeyPressFcn',@OnKeyPress);

MainAxes=axes('ButtonDownFcn',@OnLeftAxesDown); axis equal;
axis([-2,2,-2,2]);


%% Global variables
axes(MainAxes);

hold on
clickflag = false;

Solver=NewtonSolver;
hLis=addlistener(Solver,'IterationDone',@OnSolverIter);

active=false;
%% GUI initialization
BtnInitialize = gui.pushbutton('Initialize');
BtnInitialize.ValueChangedFcn = @OnInitialize;

BtnRandomize = gui.pushbutton('Randomize');
BtnRandomize.ValueChangedFcn = @OnRandomize;

TxtMenuEnergyType = gui.textmenu('Method',{'Newton','DDP'});
TxtMenuEnergyType.Value='Newton';
TxtMenuEnergyType.ValueChangedFcn = @OnUpdateParams;

sliderTimeStep = gui.intslider('Time step', [0 40]);
sliderTimeStep.ValueChangedFcn = @onSliderEigenNum;

LabelParams = gui.label('');
LabelStatus = gui.label('');

BtnLoadSource = gui.pushbutton('Check something');
BtnLoadSource.ValueChangedFcn = @OnButtonCheckSomething;



%% Callbacks

    function OnInitialize(~)
        
    end
    function OnRandomize(~)
        
    end
    function OnUpdateParams(~)

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
        LabelStatus.Value = {['Cost:', num2str(7)];
                             ['Gradient norm :', num2str(7)]};
        Redraw;
    end
    function OnButtonCheckSomething(src,evtdata)
        Redraw;
    end
%% Drawing functions
    function Redraw(~)
        % draw cars
        drawnow;
    end
end