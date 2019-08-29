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
BtnReset = gui.pushbutton('Reset');
BtnReset.ValueChangedFcn = @OnReset;

BtnInitialize = gui.togglebutton('Optimize - Preset Controls');
BtnInitialize.ValueChangedFcn = @OnInitialize;

BtnRandomize = gui.togglebutton('Optimize - Random Controls');
BtnRandomize.ValueChangedFcn = @OnRandomize;

BtnAnimateTraj = gui.pushbutton('Check and Animate Trajectories');
BtnAnimateTraj.ValueChangedFcn = @OnAnimateTraj;

MethodDropDown = gui.textmenu('Method',{'Newton and DDP','Newton','DDP'});
MethodDropDown.Value='Newton and DDP';
MethodDropDown.ValueChangedFcn = @OnUpdateParams;

%Preset configurations - These are used to populate the dropdown of presets
configKeySet = {'Diagonals','Exchange Positions', 'Exchange Positions with Initial Speed', '180° Park in Place', 'Weave', '90° Park'};
x0Arr = {[-5 -5 pi/4 0;-5  5 -pi/4 0],...
         [-5  0   0  0; 5  0  pi   0],...
         [-5  0   0  1; 5  0  pi   1],...
         [-3.5  0   0  0; 3.5  0  pi   0],...
         [-3  -6   pi/2  0; 3  -6  pi/2   0],...
         [-5  0   0  0; 5  0  pi   0]};
xTArr = {[ 5  5 pi/4 0; 5 -5 -pi/4 0],...
         [ 5  0   0  0; -5 0  pi   0],...
         [ 5  0   0  0; -5 0  pi   0],...
         [-3.5  0   pi  0; 3.5  0  0   0],...
         [3  6   pi/2  0; -3  6  pi/2   0],...
         [3  -6   3*pi/2  0; -3  -6  3*pi/2   0]};
     
keyTox0 = containers.Map(configKeySet,x0Arr)            %Maps dropdown value to config
keyToxT = containers.Map(configKeySet,xTArr)
x0 = keyTox0('Diagonals');
xT = keyToxT('Diagonals');

tableFig = figure();
tableFig.Position(3:4) = [400;200];
tableFig.Position(1:2) = [80;400];
colInitialNames = {'Initial X'; 'Initial Y'; 'Initial Angle'; 'Initial Speed'};
colTargetNames = {'Target X'; 'Target Y'; 'Target Angle'; 'Target Speed'};
rowNames = {'Car 1', 'Car 2'};

InitUITable = uitable(tableFig,'Data', x0, 'ColumnName', colInitialNames, 'ColumnFormat', {'numeric' 'numeric' 'numeric' 'numeric'}, 'RowName', rowNames);
InitUITable.ColumnEditable = true;
InitUITable.Position(2) = 100;
InitUITable.Position(3:4) = InitUITable.Extent(3:4);
TgtUITable = uitable(tableFig,'Data', xT, 'ColumnName', colTargetNames, 'ColumnFormat', {'numeric' 'numeric' 'numeric' 'numeric'}, 'RowName', rowNames);
TgtUITable.ColumnEditable = true;
TgtUITable.Position(2) = 5;
TgtUITable.Position(3:4) = TgtUITable.Extent(3:4);

InitUITable.CellEditCallback = @tableCellEdit;
TgtUITable.CellEditCallback = @tableCellEdit;

presets = configKeySet;
presets{end+1} = 'Custom';              %'Custom' is automatically chosen whenever the configuration tables are edited
PresetConfigsDropDown = gui.textmenu('Preset Configurations',presets);
PresetConfigsDropDown.Value = 'Diagonals';
PresetConfigsDropDown.ValueChangedFcn = @OnUpdateParams;

LabelParams = gui.label('');
Label1Status = gui.label('');
Label2Status = gui.label('');
Label1Status.Position.width = 400;
Label2Status.Position.width = 400;

%BtnLoadSource = gui.pushbutton('Check something');
%BtnLoadSource.ValueChangedFcn = @OnButtonCheckSomething;

%Since the solver contains a loop, it needs to be able to be able to check
%and exit appropriately when the toggle buttons change since the callbacks
%below won't be triggered. We could have everything run through callbacks
%here but then the complicated solver code would have to go here.
Solver.setButtons(BtnInitialize, BtnRandomize);
settingsUpToDate = false;

%% Callbacks
    function OnReset(hObject)
        Solver.Initialize(x0,xT,MainAxes,false);
    end
    function OnInitialize(hObject)
        if hObject.Value()
            %Only one of Initialize and Randomize may be selected
            if BtnRandomize.Value()
                BtnRandomize.Value = 0;
                Solver.StopInteractive();
                Solver.Initialize(x0,xT,MainAxes,false);
            elseif Solver.getIter() == 0 || Solver.isRandom()   %reinitialize if switching random<->preset controls
                Solver.Initialize(x0,xT,MainAxes,false);
            end
            if ~settingsUpToDate
                updateSettings(MethodDropDown.Value)
            end
            setBtnsDuringOpt(false);
            Solver.StartInteractive(MainAxes);
            BtnInitialize.Value = false;
        else
            Solver.StopInteractive();
        end
        setBtnsDuringOpt(true);
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
                Solver.Initialize(x0,xT,MainAxes,true);
            elseif Solver.getIter() == 0 || ~Solver.isRandom()
                Solver.Initialize(x0,xT,MainAxes,true);
            end
            if ~settingsUpToDate
                updateSettings(MethodDropDown.Value)
            end
            setBtnsDuringOpt(false);
            Solver.StartInteractive(MainAxes);
            BtnRandomize.Value = false;
        else
            Solver.StopInteractive();
        end
        setBtnsDuringOpt(true);
        %We might be exiting because Initialize was selected, manually
        %trigger the missed callback
        if BtnInitialize.Value()
            OnInitialize(BtnInitialize);
        end
    end
    function OnAnimateTraj(hObject)
        Solver.checkAndAnimate(MainAxes);
    end
    function OnUpdateParams(~)
        settingsUpToDate = false;
        if isKey(keyTox0, PresetConfigsDropDown.Value)      %The preset may have changed; update our initial and target configs
            x0 = keyTox0(PresetConfigsDropDown.Value);
            xT = keyToxT(PresetConfigsDropDown.Value);
            InitUITable.Data = x0;
            TgtUITable.Data = xT;
        elseif ~strcmp(PresetConfigsDropDown.Value,'Custom')        %if not a defined preset, the dropdown value must be 'Custom'
            error('Undefined Value in Preset Configuration Drop-down');
        end    
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
    function OnSolverIter(src,evtdata)                              %Update costs at every iteration
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
    function tableCellEdit(hObject,callbackData)        %Callback for editing a cell in the initial or target config tables
        numval = eval(callbackData.EditData);
        r = callbackData.Indices(1);
        c = callbackData.Indices(2);
        hObject.Data(r,c) = numval; 
        if InitUITable == hObject
            x0(r,c) = numval;
        elseif TgtUITable == hObject
            xT(r,c) = numval;
        end
        PresetConfigsDropDown.Value = 'Custom';
    end
    function OnButtonCheckSomething(src,evtdata)
        Redraw;
    end
%%Callback helpers
    function setBtnsDuringOpt(enable)
        BtnReset.Enable = enable;
        BtnAnimateTraj.Enable = enable;
        MethodDropDown.Enable = enable;
        PresetConfigsDropDown.Enable = enable;
        if enable
            InitUITable.Enable = 'on';
            TgtUITable.Enable = 'on';
        else
            InitUITable.Enable = 'off';
            TgtUITable.Enable = 'off';
        end
    end
%% Update solver settings
    function updateSettings(method)
        if strcmp(method,'Newton and DDP')
            Solver.enableNewton(true);
            Solver.enableDDP(true);
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