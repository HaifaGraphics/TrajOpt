function TrajOptGui
close all;
myGui = gui.autogui('Location','float');
myGui.PanelWidth=200;
propsP = {'LineStyle','none','Marker','o','MarkerEdge','black','MarkerFaceColor','red','MarkerSize',6};
set(gcf,'Renderer','OpenGL');

warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
jFrame = get(handle(myGui.UiHandle),'JavaFrame'); drawnow;
jFrame_fHGxClient = jFrame.fHG2Client;
jFrame_fHGxClient.getWindow.setAlwaysOnTop(true);

figColor=[0.8314,0.8157,0.7843];
% axesopt={'XTickLabel',[],'XTick',[],...
%     'YTickLabel',[],'YTick',[],...
%     'ZTickLabel',[],'ZTick',[],...
%     'Color',figColor,'XColor',figColor,'YColor',figColor,'ZColor',figColor};
axesopt={};

fh = figure('WindowKeyPressFcn',@OnKeyPress);

MainAxes=axes(axesopt{:},'ButtonDownFcn',@OnLeftAxesDown); axis equal;
axis([-2,2,-2,2]);
set(MainAxes,axesopt{:});
P=PointSet([],propsP{:});

l=addlistener(P,'PointPicked',@OnPPicked);

addlistener(P,'PointDropped',@OnPDropped);

addlistener(P,'PointDragged',@OnPointDrag);

addlistener(P,'PointRemoved',@OnPointRemoved);

%% Global variables
axes(MainAxes);

hold on
Ksh=Ks.draw('HitTest','off','PickableParts','None');
clickflag = false;

hLis=[];

Parameterizer=AutoCutsParameterizer;
hLis=addlistener(Parameterizer,'IterationDone',@OnSolverIter);

active=false;
%% GUI initialization
BtnLoadSource = gui.pushbutton('Load Mesh');
BtnLoadSource.ValueChangedFcn = @OnButtonLoadMesh;

BtnInitialize = gui.pushbutton('Initialize');
BtnInitialize.ValueChangedFcn = @OnInitialize;

BtnRandomize = gui.pushbutton('Randomize');
BtnRandomize.ValueChangedFcn = @OnRandomize;

ChkboxAvoidFlips = gui.checkbox('Avoid Flips');
ChkboxAvoidFlips.ValueChangedFcn = @OnUpdateParams;

TxtMenuEnergyType = gui.textmenu('Distortion Energy',{'ARAP','Symmetric Dirichlet'});
TxtMenuEnergyType.Value='ARAP';
TxtMenuEnergyType.ValueChangedFcn = @OnUpdateParams;

TxtMenuSeparationEnergy = gui.textmenu('Separation Energy',{'Quadratic','Sigmoid'});
TxtMenuSeparationEnergy.Value = 'Quadratic';
TxtMenuSeparationEnergy.ValueChangedFcn = @OnUpdateParams;

EditSeparationDelta = gui.editnumber('Sigmoid delta:');
EditSeparationDelta.Value = 1;
EditSeparationDelta.ValueChangedFcn = @OnUpdateParams;

EditSeparationFactor = gui.editnumber('Sigmoid weight:');
EditSeparationFactor.Value = 1;
EditSeparationFactor.ValueChangedFcn = @OnUpdateParams;

% BtnExportSource = gui.pushbutton('Export Source');
% BtnExportSource.ValueChangedFcn = @OnExportSource;
% 
% BtnExportTarget = gui.pushbutton('Export Target');
% BtnExportTarget.ValueChangedFcn = @OnExportTarget;

LabelParams = gui.label('');
LabelStatus = gui.label('');

BtnLoadSource = gui.pushbutton('Check Derivative');
BtnLoadSource.ValueChangedFcn = @OnButtonCheckDerivative;

ChkboxVisualizeDistortion = gui.checkbox('Visualize distortion');
ChkboxVisualizeDistortion.ValueChangedFcn = @Redraw;


%% Callbacks
    function OnButtonLoadMesh(~)
        [MeshFile,PathName] = uigetfile({'*.obj;*.off;*.png'});
        [~,~,ext] = fileparts(MeshFile);
        M=TriangleMesh(ext(2:end),[PathName,MeshFile],'squeeze');
        M.centralize('BoundingBox');
        Ks = Parameterizer.SetMesh(M);
        P.SetSnap(Ksh);
        figure; M.draw;
    end
    function OnInitialize(~)
        Parameterizer.Initialize;
    end
    function OnRandomize(~)
        Parameterizer.Randomize;
    end
    function OnUpdateParams(~)
        Parameterizer.FlipsFlag = ChkboxAvoidFlips.Value;
        Parameterizer.Energy.EnergyType= TxtMenuEnergyType.Value;
        Parameterizer.SeparationType = TxtMenuSeparationType.Value;
        Parameterizer.SeparationEnergy = TxtMenuSeparationEnergy.Value;
        Parameterizer.Lsep = EditSeparationFactor.Value;
        Parameterizer.delta = EditSeparationDelta.Value;
        Parameterizer.Energy.Kmax = EditMaxDist.Value;
        Parameterizer.Energy.Lb = EditDistLambda.Value;
        Parameterizer.exponentiate = ChkboxExponentiate.Value;
        Parameterizer.InitLBFGS;
        LabelParams.Value = {['Lambda: ', num2str(Parameterizer.lambda)]};
    end
    function OnPPicked(src,evt)

    end
    function OnPointDrag(~,~)
    	Parameterizer.MoveHandles(P.X');
    end
    function OnPDropped(src,evt)
    end
    function OnPointRemoved(~,~)
        Parameterizer.Fi(P.Xi)=[];
    end
    function OnLeftAxesDown(src,evtdata)
        modifier = get(gcf,'SelectionType');
        pos=get(gca,'currentpoint'); pos=pos(1,1:2)';
        switch modifier
            case 'alt'  % in windows this is 'control'
                fi=P.AddPoint(pos);
                Parameterizer.Fi=[Parameterizer.Fi, fi];
            otherwise
                clickflag = true;
                fi=P.AddPoint(pos);
                Parameterizer.Fi=[Parameterizer.Fi, fi]; 
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
            case 'd'
                i=find(strcmp(TxtMenuEnergyType.Value,TxtMenuEnergyType.MenuItems));
                TxtMenuEnergyType.Value = TxtMenuEnergyType.MenuItems{mod(i,numel(TxtMenuEnergyType.MenuItems))+1};
            case '>'
                if Parameterizer.lambda<0.95
                    Parameterizer.lambda = Parameterizer.lambda+0.1;
                    Parameterizer.InitLBFGS
                end
            case '<'
                if Parameterizer.lambda>0.05
                    Parameterizer.lambda = Parameterizer.lambda-0.1;
                    Parameterizer.InitLBFGS
                end
            case 'K'
                    EditSeparationDelta.Value = EditSeparationDelta.Value/2;
            case 'L'
                    EditSeparationDelta.Value = EditSeparationDelta.Value*2;
        end
        OnUpdateParams;
    end
    function OnSolverIter(src,evtdata)
        Ks.V=Parameterizer.P';
        LabelStatus.Value = {['Step:', num2str(Parameterizer.t)];
                              ['Grad:', num2str(norm(Parameterizer.Gradient))];
                              ['Max Dist ', num2str(max(sum(Parameterizer.Energy.fi.^2,2)))]};
        Redraw;
    end
    function OnExportTarget(~)
        [FileName,PathName] = uiputfile('Results\*.*');
        State=States{SldStateNum.Value};
        [~,name,ext] = fileparts(FileName);
        save([PathName,name],'State');
        SaveTarget([PathName,name]);
    end
    function OnExportSource(~)
        [FileName,PathName] = uiputfile('Results\*.*');
        SaveSource([PathName,FileName]);
    end
    function OnButtonCheckDerivative(~)
        Parameterizer.CheckDerivative;
    end
%% Drawing functions
    function Redraw(~)
        if ~ChkboxVisualizeDistortion.Value
            Ks.draw(Ksh);
        else
            D = sum(Parameterizer.Energy.fi.^2,2);
            Ks.draw(Ksh,'FaceColor','flat','CData',D);
        end
        drawnow;
    end
end