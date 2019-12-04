classdef GUI_Beta_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        File                            matlab.ui.container.Menu
        tabGroupMain                    matlab.ui.container.TabGroup
        tabRobot                        matlab.ui.container.Tab
        panelRobot                      matlab.ui.container.Panel
        AddNewLinkButton                matlab.ui.control.Button
        RemoveLinkButton                matlab.ui.control.Button
        UpdateLinkButton                matlab.ui.control.Button
        VisualzeRobotButton             matlab.ui.control.Button
        EditFullRobotButton             matlab.ui.control.Button
        ExamplePumaButton               matlab.ui.control.Button
        ExampleStanfordButton           matlab.ui.control.Button
        Example2LinkButton              matlab.ui.control.Button
        panelLink                       matlab.ui.container.Panel
        tabGroupLinkDef                 matlab.ui.container.TabGroup
        tabLinkKine                     matlab.ui.container.Tab
        panelLinkDefOptions             matlab.ui.container.Panel
        GridLayout                      matlab.ui.container.GridLayout
        GeometryLabel                   matlab.ui.control.Label
        dropDownGeometry                matlab.ui.control.DropDown
        JointTypeDropDownLabel          matlab.ui.control.Label
        dropDownJointType               matlab.ui.control.DropDown
        panelGeometry                   matlab.ui.container.Panel
        GridLayout2                     matlab.ui.container.GridLayout
        EditFieldLabel                  matlab.ui.control.Label
        fieldKineDOrZDir                matlab.ui.control.NumericEditField
        EditField2Label                 matlab.ui.control.Label
        fieldKineThetaOrLength          matlab.ui.control.NumericEditField
        EditField3Label                 matlab.ui.control.Label
        fieldKineAOrVertOffset          matlab.ui.control.NumericEditField
        EditField4Label                 matlab.ui.control.Label
        fieldKineAlphaOrAngOffset       matlab.ui.control.NumericEditField
        JointMinimimumEditFieldLabel    matlab.ui.control.Label
        fieldJointMin                   matlab.ui.control.NumericEditField
        JointMaximumEditFieldLabel      matlab.ui.control.Label
        fieldJointMax                   matlab.ui.control.NumericEditField
        tabLinkDynamics                 matlab.ui.container.Tab
        GridLayout3                     matlab.ui.container.GridLayout
        GridCOGPos                      matlab.ui.container.GridLayout
        COG1                            matlab.ui.control.NumericEditField
        COG2                            matlab.ui.control.NumericEditField
        COG3                            matlab.ui.control.NumericEditField
        GridInertiaMatrix               matlab.ui.container.GridLayout
        Inertia11                       matlab.ui.control.NumericEditField
        Inertia12                       matlab.ui.control.NumericEditField
        Inertia13                       matlab.ui.control.NumericEditField
        Inertia21                       matlab.ui.control.NumericEditField
        Inertia31                       matlab.ui.control.NumericEditField
        Inertia22                       matlab.ui.control.NumericEditField
        Inertia32                       matlab.ui.control.NumericEditField
        Inertia23                       matlab.ui.control.NumericEditField
        Inertia33                       matlab.ui.control.NumericEditField
        labelCenterofGravityLocation    matlab.ui.control.Label
        labelInertiaMatrixaboutCenterofGravity  matlab.ui.control.Label
        LinkMassEditFieldLabel          matlab.ui.control.Label
        fieldDynLinkMass                matlab.ui.control.NumericEditField
        ViscousFrictionEditFieldLabel   matlab.ui.control.Label
        fieldDynViscousFriction         matlab.ui.control.NumericEditField
        CoulombFrictionEditFieldLabel   matlab.ui.control.Label
        fieldDynCoulombFriction         matlab.ui.control.NumericEditField
        GearRatioEditFieldLabel         matlab.ui.control.Label
        fieldDynGearRatio               matlab.ui.control.NumericEditField
        MotorInertiaEditFieldLabel      matlab.ui.control.Label
        fieldDynMotorInertia            matlab.ui.control.NumericEditField
        listBoxRobot                    matlab.ui.control.ListBox
        CoordinateTransformTab          matlab.ui.container.Tab
        GridLayout23                    matlab.ui.container.GridLayout
        axesTranf                       matlab.ui.control.UIAxes
        GridLayout24                    matlab.ui.container.GridLayout
        HomogenousTransformationPanel   matlab.ui.container.Panel
        GridLayout25                    matlab.ui.container.GridLayout
        tranfTable                      matlab.ui.control.Table
        Panel_2                         matlab.ui.container.Panel
        GridLayout26                    matlab.ui.container.GridLayout
        TabGroup4                       matlab.ui.container.TabGroup
        ConvertTab                      matlab.ui.container.Tab
        GridLayout27                    matlab.ui.container.GridLayout
        converterTable                  matlab.ui.control.Table
        GridLayout28                    matlab.ui.container.GridLayout
        ConverttoTransformLabel         matlab.ui.control.Label
        ConvertToTDropDown              matlab.ui.control.DropDown
        GridLayout29                    matlab.ui.container.GridLayout
        ClearButton                     matlab.ui.control.Button
        VisualizeSlideTab               matlab.ui.container.Tab
        GridLayout30                    matlab.ui.container.GridLayout
        RzRotationSliderLabel           matlab.ui.control.Label
        RzRotationSlider                matlab.ui.control.Slider
        RyRotationSliderLabel           matlab.ui.control.Label
        RyRotationSlider                matlab.ui.control.Slider
        RxRotationSliderLabel           matlab.ui.control.Label
        RxRotationSlider                matlab.ui.control.Slider
        xTranslationSliderLabel         matlab.ui.control.Label
        xTranslationSlider              matlab.ui.control.Slider
        yTranslationSliderLabel         matlab.ui.control.Label
        yTranslationSlider              matlab.ui.control.Slider
        zTranslationSliderLabel         matlab.ui.control.Label
        zTranslationSlider              matlab.ui.control.Slider
        WorkspaceTab                    matlab.ui.container.Tab
        WorkspaceOptionsPanel           matlab.ui.container.Panel
        GridLayout4                     matlab.ui.container.GridLayout
        GenerateButton                  matlab.ui.control.Button
        MeshResolutionEditFieldLabel    matlab.ui.control.Label
        MeshResolutionEditField         matlab.ui.control.NumericEditField
        VisualiziationPanel             matlab.ui.container.Panel
        axesWorkspace                   matlab.ui.control.UIAxes
        TextArea                        matlab.ui.control.TextArea
        KinematicsTab                   matlab.ui.container.Tab
        TabGroup2                       matlab.ui.container.TabGroup
        ForwardKinematicsTab            matlab.ui.container.Tab
        GridLayout19                    matlab.ui.container.GridLayout
        KinematicAnimationPanel         matlab.ui.container.Panel
        GridLayout6                     matlab.ui.container.GridLayout
        AnimateKineButton               matlab.ui.control.Button
        GivePosesButton                 matlab.ui.control.Button
        FindSingularitiesButton         matlab.ui.control.Button
        ShowPosesButton                 matlab.ui.control.Button
        JointConfigurationsPanel        matlab.ui.container.Panel
        GridLayout20                    matlab.ui.container.GridLayout
        kineConfigTable                 matlab.ui.control.Table
        GridLayout33                    matlab.ui.container.GridLayout
        ForwardKinematicsOptionsPanel   matlab.ui.container.Panel
        GridLayout5                     matlab.ui.container.GridLayout
        ConfigurationNumberEditFieldLabel  matlab.ui.control.Label
        ConfigurationNumberEditField    matlab.ui.control.NumericEditField
        DifferentialKinematicsOptionsPanel  matlab.ui.container.Panel
        GridLayout34                    matlab.ui.container.GridLayout
        SampleFrequencyEditFieldLabel   matlab.ui.control.Label
        SampleFrequencyEditField        matlab.ui.control.NumericEditField
        FinalTimeEditField_2Label       matlab.ui.control.Label
        diffFinalTimeEditField          matlab.ui.control.NumericEditField
        SingularitiesPanel              matlab.ui.container.Panel
        GridLayout35                    matlab.ui.container.GridLayout
        singularTable                   matlab.ui.control.Table
        InverseKinematicsTab            matlab.ui.container.Tab
        GridLayout21                    matlab.ui.container.GridLayout
        InverseKinematicAnimationPanel  matlab.ui.container.Panel
        GridLayout13                    matlab.ui.container.GridLayout
        AnimateInvKineButton            matlab.ui.control.Button
        AnimateInvKineJacoButton        matlab.ui.control.Button
        GiveConfigurationsButton        matlab.ui.control.Button
        ShowConfigurationsButton        matlab.ui.control.Button
        EndEffectorPosePanel            matlab.ui.container.Panel
        GridLayout22                    matlab.ui.container.GridLayout
        invKineConfigTable              matlab.ui.control.Table
        GridLayout36                    matlab.ui.container.GridLayout
        InverseKinematicsOptionsPanel   matlab.ui.container.Panel
        GridLayout7                     matlab.ui.container.GridLayout
        PoseNumberLabel                 matlab.ui.control.Label
        PoseNumberEditField             matlab.ui.control.NumericEditField
        DifferentialKinematicsOptionsPanel_2  matlab.ui.container.Panel
        GridLayout34_2                  matlab.ui.container.GridLayout
        SampleFrequencyEditField_3Label  matlab.ui.control.Label
        SampleFrequencyEditField_3      matlab.ui.control.NumericEditField
        FinalTimeEditField_2Label_3     matlab.ui.control.Label
        diffFinalTimeEditField_3        matlab.ui.control.NumericEditField
        DynamicsTab                     matlab.ui.container.Tab
        InputsPanel                     matlab.ui.container.Panel
        FinalTimeLabel                  matlab.ui.control.Label
        FinalTimeEditField              matlab.ui.control.NumericEditField
        InitialPositionPanel            matlab.ui.container.Panel
        GridLayout9                     matlab.ui.container.GridLayout
        q0Table                         matlab.ui.control.Table
        TorqueFunctionOptionsPanel      matlab.ui.container.Panel
        TypeDropDownLabel               matlab.ui.control.Label
        TypeDropDown                    matlab.ui.control.DropDown
        NameEditFieldLabel              matlab.ui.control.Label
        NameEditField                   matlab.ui.control.EditField
        InitialVelocityPanel            matlab.ui.container.Panel
        GridLayout10                    matlab.ui.container.GridLayout
        qd0Table                        matlab.ui.control.Table
        TabGroup                        matlab.ui.container.TabGroup
        JointPositionsandVelocitiesTab  matlab.ui.container.Tab
        DynamicsPlotsPanel              matlab.ui.container.Panel
        GridLayout8                     matlab.ui.container.GridLayout
        axesJointPositions              matlab.ui.control.UIAxes
        axesJointVelocities             matlab.ui.control.UIAxes
        EquationsofMotionTab            matlab.ui.container.Tab
        GridLayout31                    matlab.ui.container.GridLayout
        Image                           matlab.ui.control.Image
        eqMotionInputTable              matlab.ui.control.Table
        GridLayout32                    matlab.ui.container.GridLayout
        EOMSelectDropDown               matlab.ui.control.DropDown
        DisplaySelectedMatrixButton     matlab.ui.control.Button
        eqMotionOutputTable             matlab.ui.control.Table
        ControlPanel                    matlab.ui.container.Panel
        CalculateDynamicsButton         matlab.ui.control.Button
        AnimateDynamicsButton           matlab.ui.control.Button
        InputsPanel_2                   matlab.ui.container.Panel
        GridLayout11                    matlab.ui.container.GridLayout
        torqFunTable                    matlab.ui.control.Table
        ControlTab                      matlab.ui.container.Tab
        Panel                           matlab.ui.container.Panel
        GridLayout12                    matlab.ui.container.GridLayout
        ConsoleTextArea                 matlab.ui.control.TextArea
    end

    
    properties (Access = public)
        baseLink = Link % default base link
        robot = SerialLink() % makes a null robot
        
        % vector to store nodes for the robot tree (treebot, if you will)
        treeRobotNodes = []
        % Note: Should mirror the robot.links property at all times
        
        % dyanmics tab
        q0FieldsVals = [];
        qd0FieldsVals = [];
        
        grey = '#555555';
        white = '#000000';
        
        
    end

    
    methods (Access = private)
        
        %% UI Functions
        
        % Robot Tab
        function populateListBoxRobot(app)
        % populates the tree with the SerialLink object's link
            for i = 1:length(app.robot.links)
                listBoxItems(i) = "Link " + num2str(i);
                listBoxItemsData(i) = i;
            end
            
            app.listBoxRobot.Items = listBoxItems;
            app.listBoxRobot.ItemsData = listBoxItemsData;
            
        end
        
        function populateTableGeometry(app)
            % method for populating the robot geometry table based on
            % inputs.
            
            %% Kinematics Tab
            if app.dropDownGeometry.Value == "DH Parameters"
                app.EditFieldLabel.Text = "D";
                app.EditField2Label.Text = "Theta";
                app.EditField3Label.Text = "A";
                app.EditField4Label.Text = "Alpha";
                if app.dropDownJointType.Value == "Revolute"   
                    app.fieldKineThetaOrLength.Editable = 0;
                    app.fieldKineThetaOrLength.FontColor = app.grey; % should be a greyish color
                    app.fieldKineDOrZDir.Editable = 1;
                    app.fieldKineDOrZDir.FontColor = app.white; 
                    app.fieldKineAOrVertOffset.Editable = 1;
                    app.fieldKineAOrVertOffset.FontColor = app.white; 
                    app.fieldJointMin.Value = -pi;
                    app.fieldJointMax.Value = pi;
                    
                else % other option is prismatic
                    app.fieldKineThetaOrLength.Editable = 1;
                    app.fieldKineThetaOrLength.FontColor = app.white; 
                    app.fieldKineDOrZDir.Editable = 0;
                    app.fieldKineDOrZDir.FontColor = app.grey; % should be a greyish color
                    app.fieldKineAOrVertOffset.Editable = 1;
                    app.fieldKineAOrVertOffset.FontColor = app.white;
                    app.fieldJointMin.Value = -5;
                    app.fieldJointMax.Value = 5;
                end
              
            else % general geometric description
                app.EditFieldLabel.Text = "Joint Direction Offset";
                app.EditField2Label.Text = "Twist About Joint Direction";
                app.EditField3Label.Text = "Link Length";
                app.EditField4Label.Text = "Angle Between Joint Directions";
               if app.dropDownJointType.Value == "Revolute"
                   app.fieldKineDOrZDir.Editable = 1;
                    app.fieldKineDOrZDir.FontColor = app.white; 
                    app.fieldKineThetaOrLength.Editable = 0;
                    app.fieldKineThetaOrLength.FontColor = app.grey; % should be a greyish color
                    app.fieldKineAOrVertOffset.Editable = 1;
                    app.fieldKineAOrVertOffset.FontColor = app.white; 
                    app.fieldJointMin.Value = -pi;
                    app.fieldJointMax.Value = pi;
               else % other option is prismatic
                    app.fieldKineThetaOrLength.Editable = 1;
                    app.fieldKineThetaOrLength.FontColor = app.white; 
                    app.fieldKineDOrZDir.Editable = 0;
                    app.fieldKineDOrZDir.FontColor = app.grey;
                    app.fieldKineAOrVertOffset.Editable = 1;
                    app.fieldKineAOrVertOffset.FontColor = app.white; % should be a greyish color
                    app.fieldJointMin.Value = -5;
                    app.fieldJointMax.Value = 5;
               end
            end
            
        end
        
        function updateGeometryTable(app)
            % updates Geoemetry table based on which link was selected
            index = app.getCurrentLinkIndex();
            app.print("Updating Geoemtry Table with Link " + num2str(index) + "'s information...");
            
            %% Kinematics
            if app.robot.links(index).isrevolute()
                app.fieldKineDOrZDir.Value = app.robot.links(index).d;
                app.fieldKineThetaOrLength.Value = 0;
            else
                app.fieldKineDOrZDir.Value = 0;
                app.fieldKineThetaOrLength.Value = app.robot.links(index).theta;
            end
            
            app.fieldKineAOrVertOffset.Value = app.robot.links(index).a;
            app.fieldKineAlphaOrAngOffset.Value = app.robot.links(index).alpha;
            
            %% Dynamics
            app.COG1.Value = app.robot.links(index).r(1);
            app.COG2.Value = app.robot.links(index).r(2);
            app.COG3.Value = app.robot.links(index).r(3);
            app.Inertia11.Value = app.robot.links(index).I(1,1);
            app.Inertia12.Value = app.robot.links(index).I(1,2);
            app.Inertia13.Value = app.robot.links(index).I(1,3);
            app.Inertia21.Value = app.robot.links(index).I(2,1);
            app.Inertia22.Value = app.robot.links(index).I(2,2);
            app.Inertia23.Value = app.robot.links(index).I(2,3);
            app.Inertia31.Value = app.robot.links(index).I(3,1);
            app.Inertia32.Value = app.robot.links(index).I(3,2);
            app.Inertia33.Value = app.robot.links(index).I(3,3);
            app.fieldDynLinkMass.Value = app.robot.links(index).m;
            app.fieldDynViscousFriction.Value = app.robot.links(index).B;
            app.fieldDynGearRatio.Value = app.robot.links(index).G;
            app.fieldDynMotorInertia.Value = app.robot.links(index).Jm;
        end
        
        % Dynamics Tab
        
        function populateDynInitialConditions(app)
            % creates propertly sized grids for inputting initial
            % conditions
%             boxHeight = 20; % in pixels, I think
%             boxWidth = 60;
%             leftSpacing = 20;
%             padding = 10;
%             
%             app.q0FieldsVals = [];
%             app.qd0FieldsVals = [];
%             
%             % fill in the tab
%             for i=length(app.robot.links):-1:1 % it's backwards because matlab is garbage
%                 disp(i);
%                 bottomSpacing = i*(padding + boxHeight) - padding;
%                 posLabel = [leftSpacing, bottomSpacing, boxWidth, boxHeight];
%                 posField = [leftSpacing + boxWidth + padding, bottomSpacing, boxWidth, boxHeight];
%                 uilabel(app.InitialPositionPanel,'Text', "Joint "+ num2str(i), 'Position', posLabel);
%                 uilabel(app.InitialVelocityPanel, 'Text', "Joint "+ num2str(i), 'Position', posLabel);
%                 
%                 app.q0FieldsVals(i) = uieditfield(app.InitialPositionPanel, 'numeric', 'Value', 0,'Position', posField);
%                 app.qd0FieldsVals(i) = uieditfield(app.InitialVelocityPanel, 'numeric', 'Value', 0, 'Position', posField);
%                 
%             end
            linkNum = length(app.robot.links);
            
            app.q0Table.Data = zeros(1,linkNum);
            app.qd0Table.Data = zeros(1,linkNum);
        end
        
        %% Robot Editing Functions
        function addLink(app)
        % make a new link and put it after the currently selected one
        app.robot =  SerialLink([app.robot.links [app.getLinkFromPanel()]]);
        end
        
        function removeLink(app)
        % removed the currently selected link if able
        linkIndex = app.getCurrentLinkIndex();
        if linkIndex ~= 1
            if linkIndex ~= length(app.robot.links) % if not last link
                beforeLink = app.robot.links(1:linkIndex-1);
                afterLink = app.robot.links(linkIndex+1:end);
                app.robot = SerialLink([beforeLink afterLink]);
            else % if last link
                app.robot = SerialLink([app.robot.links(1:linkIndex-1)]);
            end
        else % if selected link is base link or root node
            app.print("Cannot remove base link!");
        end
        
        end
        
        function updateLink(app)
            % Make a new link with current values and replace the old link
            % with it
            linkIndex = app.getCurrentLinkIndex();
            linkVec = app.robot.links;
            linkVec(linkIndex) = app.getLinkFromPanel();
            app.robot = SerialLink(linkVec);
           
        end
        
        %% Helper Functions
        
        function print(app, str)
            preFormatted = "\n" + str;
            formattedStr = sprintf(preFormatted);
            app.ConsoleTextArea.Value = formattedStr;
        end
        
        
        function [n] = getCurrentLinkIndex(app)
            n = app.listBoxRobot.Value; % gives the item data associated with selected box
        end
        
        function [newLink] = getLinkFromPanel(app)
            % make link with geometry depending on panel selections
            if app.dropDownGeometry.Value == "DH Parameters"
                a = app.fieldKineAOrVertOffset.Value;
                alph = app.fieldKineAlphaOrAngOffset.Value;
                if app.dropDownJointType.Value == "Revolute"
                    d = app.fieldKineDOrZDir.Value;
                    newLink = Link('revolute', 'd', d, 'a', a, 'alpha', alph, 'qlim',[app.fieldJointMin.Value, app.fieldJointMax.Value]);
                    
                else % if prismatic
                    theta = app.fieldKineThetaOrLength.Value;
                    newLink = Link('prismatic', 'theta', theta, 'a', a, 'alpha', alph, 'qlim', [app.fieldJointMin.Value, app.fieldJointMax.Value]);
                end
                
            else % if geometric description
                a = app.fieldKineAOrVertOffset.Value;
                alph = app.fieldKineAlphaOrAngOffset.Value;
                if app.dropDownJointType.Value == "Revolute"
                    d = app.fieldKineDOrZDir.Value;
                    newLink = Link('revolute', 'd', d, 'a', a, 'alpha', alph, 'qlim',[app.fieldJointMin.Value, app.fieldJointMax.Value]);
                    
                else % if prismatic
                    theta = app.fieldKineThetaOrLength.Value;
                    newLink = Link('prismatic', 'theta', theta, 'a', a, 'alpha', alph, 'qlim', [app.fieldJointMin.Value, app.fieldJointMax.Value]);
                end
            end
            
            % add dynamic parameters to the link
            newLink.m = app.fieldDynLinkMass.Value;
            newLink.r = [app.COG1.Value app.COG2.Value app.COG3.Value];
            newLink.I = [app.Inertia11.Value app.Inertia12.Value app.Inertia13.Value;
                         app.Inertia21.Value app.Inertia22.Value app.Inertia23.Value;
                         app.Inertia31.Value app.Inertia32.Value app.Inertia33.Value];
            newLink.B = app.fieldDynViscousFriction.Value;
            newLink.Tc = app.fieldDynCoulombFriction.Value;
            newLink.G = app.fieldDynGearRatio.Value;
            newLink.Jm = app.fieldDynMotorInertia.Value;

        end
        
        function [q0, qd0] = getDynInitialConditions(app)
            % gets initial conditions from GUI, returns them in row vectors
            q0 = app.q0Table.Data;
            qd0 = app.qd0Table.Data;
        end
        
        function updateTable(~, table, rowNum, colNum)
            oldData = table.Data;
            if size(oldData,1) < rowNum && colNum == size(oldData,2)
               table.Data = [table.Data; zeros(rowNum - size(oldData,1), colNum)];
            elseif colNum ~= size(oldData,2)
               table.Data = zeros(rowNum, colNum);
            else
               table.Data = table.Data(1:rowNum, :); 
            end
        end
        
        function updateJointNum(app)
            jointNum = length(app.robot.links);
            names = cell(1,jointNum);
            for i = 1:jointNum
                names{i} = "Joint " + num2str(i);
            end
            app.kineConfigTable.ColumnName = names;
            
            % kinematics tab
            rowNum = app.ConfigurationNumberEditField.Value;
            colNum = jointNum;
            app.updateTable(app.kineConfigTable, rowNum, colNum);
            
            rowNum = app.PoseNumberEditField.Value;
            app.invKineConfigTable.ColumnName = {'x', 'y', 'z', 'R', 'P', 'Y'};
            colNum = 6;
            app.updateTable(app.invKineConfigTable, rowNum, colNum);
            
            % dynamics tab
            app.q0Table.ColumnName = names;
            app.qd0Table.ColumnName = names;
            app.populateDynInitialConditions();
        end
        
        function safePlot(app, q, varargin)
            % adjusts for workspaces when prismatic joints become involved
            
            % finds if there's a prismatic joint
            jointNum = length(app.robot.links);
            indices = zeros(1,jointNum);
            for i = 1:jointNum
                flag = 0;
                if app.robot.links(i).isprismatic()
                    flag = 1;
                end
                indices(i) = flag;
            end
            if max(indices) ~= 0
                bigDim = max(app.robot.links(boolean(indices)).qlim);
                wkSpce = [-bigDim bigDim -bigDim bigDim -bigDim bigDim];
                app.robot.plot(q,'workspace', wkSpce, varargin{:});
            else
                app.robot.plot(q, varargin{:});
            end
        end
        
        function updateDynInputTable(app)
            jointNum = length(app.robot.links);
            % changes what each torque function needs
            if app.TypeDropDown.Value == "Constant"
                % needs col for each joint
                colNames = cell(1,jointNum);
                for i=1:jointNum
                    colNames{i} = "Joint " + num2str(i);
                end
                
                
                app.torqFunTable.ColumnName = colNames;
                app.torqFunTable.ColumnEditable = true;
                app.NameEditField.FontColor = app.grey;
                
                app.updateTable(app.torqFunTable, 1, jointNum);
            elseif app.TypeDropDown.Value == "PD"
                % needs qstar, p, d (joints plus two cols)
                
                colNames = cell(1,jointNum+2);
                for i=1:jointNum
                    colNames{i} = "Joint " + num2str(i);
                end
                colNames{jointNum + 1} = "P";
                colNames{jointNum + 2} = "D";
                
                app.updateTable(app.torqFunTable, 1, jointNum+2);
                app.torqFunTable.ColumnName = colNames;
                app.torqFunTable.ColumnEditable = true;
                app.NameEditField.FontColor = app.grey;
            elseif app.TypeDropDown.Value == "Custom"
                % going to make unuseable
                app.torqFunTable.ColumnName = "";
                app.torqFunTable.ColumnEditable = false;
                app.torqFunTable.ForegroundColor = app.grey;
                app.NameEditField.FontColor = app.white;
                app.updateTable(app.torqFunTable, 0, jointNum);
            end
        end
        
        function populateCoordinateTransform(app)
            if app.ConvertToTDropDown.Value == "RPY"
                   app.converterTable.ColumnName = {'R', 'P', 'Y'};
                   app.updateTable(app.converterTable, 1, 3);
                   
            elseif app.ConvertToTDropDown.Value == "ZYZ"
                   app.converterTable.ColumnName = {'Z', 'Y', 'Z'};
                   app.updateTable(app.converterTable, 1, 3);
                   
            app.updateTable(app.tranfTable, 4, 4);
 

                    
%                 elseif app.ConvertToTDropDown.Value == "DH"
%                     
%                     
            end
        end
        
        function updateCoordinateTransform(app, source)
            TOld = app.tranfTable.Data;
            if isempty(TOld)
                TOld = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            end
            if strcmp(source, 'T')
                % update convert table
                if app.ConvertToTDropDown.Value == "RPY"
                    app.converterTable.Data = tr2rpy(app.tranfTable.Data);        
                    
                elseif app.ConvertToTDropDown.Value == "ZYZ"
                    app.converterTable.Data = tr2eul(app.tranfTable.Data);
                    
%                 elseif app.ConvertToTDropDown.Value == "DH"
%                     
%                     
                end
                
            end
            
            if strcmp(source, 'Convert')
                % update T table
                if app.ConvertToTDropDown.Value == "RPY"
                    T = rpy2tr(app.converterTable.Data);
                    app.tranfTable.Data = T;
                    
                elseif app.ConvertToTDropDown.Value == "ZYZ"
                    T = eul2tr(app.converterTable.Data);
                    app.tranfTable.Data = T;
                    
%                 elseif app.ConvertToTDropDown.Value == "DH"
%                     T
                 end
                
            end
            
            if strcmp(source, 'Sliders')
                % use sliders to make a transformation matrix
                Rz = trotz(app.RzRotationSlider.Value);
                Ry = troty(app.RyRotationSlider.Value);
                Rx = trotx(app.RxRotationSlider.Value);
                Trot = Rz*Ry*Rx;
                Trot(1:3,4) = [app.xTranslationSlider.Value; app.yTranslationSlider.Value; app.zTranslationSlider.Value];
                app.tranfTable.Data = Trot;
            end
            
            % get transformation
            T = app.tranfTable.Data;
            
            % plot transform
            base = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            newFrame = T*base;
            cla(app.axesTranf);

            maxLim = 2;
            disp(maxLim);
            
            trplot(base, 'rgb', 'handle', app.axesTranf);
            
            %trplot(newFrame, 'rgb', 'handle', app.axesTranf);
            set(app.axesTranf, 'PlotBoxAspectRatioMode', 'manual');
            set(app.axesTranf, 'PlotBoxAspectRatio', [1 1 1]);
            set(app.axesTranf, 'XGrid', 'on');
            set(app.axesTranf, 'YGrid', 'on');
            set(app.axesTranf, 'ZGrid', 'on');
            set(app.axesTranf, 'XLimMode', 'manual');
            set(app.axesTranf, 'YLimMode', 'manual');
            set(app.axesTranf, 'ZLimMode', 'manual');
            set(app.axesTranf, 'XLim', [-maxLim,maxLim]);
            set(app.axesTranf, 'YLim', [-maxLim,maxLim]);
            set(app.axesTranf, 'ZLim', [-maxLim,maxLim]);
            app.axesTranf.Title.String = 'Transformation Visualiztion';
            try
                tranimate(TOld, newFrame, 'rgb', 'handle', app.axesTranf, 'nsteps', 20);
            catch ME
                app.print("Plotting issue, fixing it...");
                app.ClearButtonPushed();
            end
        end
        
        function populateEOM(app)
            app.eqMotionInputTable.Data = zeros(2, length(app.robot.links));
            app.eqMotionInputTable.RowName = {'q', 'qd'};
            for i=length(app.robot.links)
                names{i} = "Joint " + num2str(i);
            end
            app.eqMotionInputTable.ColumnName = names;
            app.eqMotionOutputTable.ColumnName = {};
        end
        
        function updateEOM(app)
            % need q and qd
            q = app.eqMotionInputTable.Data(1,:);
            qd = app.eqMotionInputTable.Data(2,:);
            selectedMatrix = app.EOMSelectDropDown.Value;
            outMatrix = eqsOfMotion(app.robot, q, qd, 'ee');
            app.eqMotionOutputTable.Data = outMatrix(selectedMatrix);
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupCallback(app)
            % Callback to be ran on app startup. Should initialize all
            % default values.
            
            %% Robot Tab
            
            % fill robot definitions with default options
            app.populateTableGeometry();
            
            % Initialize default link and default SerialLink robot
            app.baseLink = app.getLinkFromPanel();
            app.robot = SerialLink([app.baseLink]);
            
            % give null robot the baselink
            app.robot.name = "Default Robot";
            
            % populate treeRobot with default robot
            app.populateListBoxRobot();
            app.updateJointNum();
            
            %% Homotranf tab
            app.tranfTable.Data = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            app.populateCoordinateTransform();
            
            % kinematics tab
            rowNum = app.ConfigurationNumberEditField.Value;
            colNum = length(app.robot.links);
            app.updateTable(app.kineConfigTable, rowNum, colNum);
            
            %% Dynamics Tab
           
           
            
        end

        % Selection change function: tabGroupMain
        function tabGroupMainSelectionChanged(app, event)
            selectedTab = app.tabGroupMain.SelectedTab;
            
            %% Kinematics Tab
            app.updateJointNum();
            
            %% Dynamics Tab
            if strcmp(selectedTab.Title, 'Dynamics')
                if app.TypeDropDown.Value ~= "Custom"
                    app.NameEditField.Editable = 0;
                    app.NameEditField.FontColor = app.grey;
                else
                    app.NameEditField.Editable = 1;
                    app.NameEditField.FontColor = app.white;
                end
                app.populateDynInitialConditions();
                app.updateDynInputTable();
            end
            
            %% Homogenous Tranform Tab
            if strcmp(selectedTab.Title, 'Coordinate Transform')
                app.updateTable(app.tranfTable, 4, 4);
            end
            
        end

        % Callback function
        function treeRobotSelectionChanged(app, event)
            % callback for changing selected nodes on the treeRobot tree
            % Should update the link description area with information
            % about that node
            
            
            % find out which treeNode is selected and update panels
            % accordingly
            
            
        end

        % Value changed function: dropDownGeometry
        function dropDownGeometryValueChanged(app, event)
            app.populateTableGeometry();
        end

        % Value changed function: dropDownJointType
        function dropDownJointTypeValueChanged(app, event)
            app.populateTableGeometry();
        end

        % Button pushed function: AddNewLinkButton
        function AddNewLinkButtonPushed(app, event)
            % Adds a new link after selected link (if L is selected links,
            % puts this link as link L+1)
            app.addLink();
            app.populateListBoxRobot();
            app.updateJointNum();
        end

        % Button pushed function: RemoveLinkButton
        function RemoveLinkButtonPushed(app, event)
            % Removes current selected link
            app.removeLink();
            app.populateListBoxRobot();
            app.updateJointNum();
        end

        % Button pushed function: UpdateLinkButton
        function UpdateLinkButtonPushed(app, event)
            % Updates parameters of selected link
            app.updateLink();
            app.populateListBoxRobot();
            app.updateJointNum();
            
            % update tabs accordingly
            app.populateDynInitialConditions();
        end

        % Button pushed function: EditFullRobotButton
        function EditFullRobotButtonPushed(app, event)
            % Opens table of robot parameters for each link
            app.robot.edit();
            app.populateListBoxRobot();
            app.updateJointNum();
        end

        % Button pushed function: VisualzeRobotButton
        function VisualzeRobotButtonPushed(app, event)
        % Uses the teach method to give a basic visualizion of what user has made
            app.safePlot(zeros([1, length(app.robot.links)]));
            app.robot.teach();
        end

        % Button pushed function: ExamplePumaButton
        function ExamplePumaButtonPushed(app, event)
            %MDL_PUMA560 Create model of Puma 560 manipulator
            %
            % MDL_PUMA560 is a script that creates the workspace variable p560 which
            % describes the kinematic and dynamic characteristics of a Unimation Puma
            % 560 manipulator using standard DH conventions. 
            %
            % Also define the workspace vectors:
            %   qz         zero joint angle configuration
            %   qr         vertical 'READY' configuration
            %   qstretch   arm is stretched out in the X direction
            %   qn         arm is at a nominal non-singular configuration
            %
            % Notes::
            % - SI units are used.
            % - The model includes armature inertia and gear ratios.
            %
            % Reference::
            % - "A search for consensus among model parameters reported for the PUMA 560 robot",
            %   P. Corke and B. Armstrong-Helouvry, 
            %   Proc. IEEE Int. Conf. Robotics and Automation, (San Diego), 
            %   pp. 1608-1613, May 1994.
            %
            % See also SerialRevolute, mdl_puma560akb, mdl_stanford.
            
            % MODEL: Unimation, Puma560, dynamics, 6DOF, standard_DH
            
            %
            % Notes:
            %    - the value of m1 is given as 0 here.  Armstrong found no value for it
            % and it does not appear in the equation for tau1 after the substituion
            % is made to inertia about link frame rather than COG frame.
            % updated:
            % 2/8/95  changed D3 to 150.05mm which is closer to data from Lee, AKB86 and Tarn
            %  fixed errors in COG for links 2 and 3
            % 29/1/91 to agree with data from Armstrong etal.  Due to their use
            %  of modified D&H params, some of the offsets Ai, Di are
            %  offset, and for links 3-5 swap Y and Z axes.
            % 14/2/91 to use Paul's value of link twist (alpha) to be consistant
            %  with ARCL.  This is the -ve of Lee's values, which means the
            %  zero angle position is a righty for Paul, and lefty for Lee.
            %  Note that gravity load torque is the motor torque necessary
            %  to keep the joint static, and is thus -ve of the gravity
            %  caused torque.
            %
            % 8/95 fix bugs in COG data for Puma 560. This led to signficant errors in
            %  inertia of joint 1. 
            % $Log: not supported by cvs2svn $
            % Revision 1.4  2008/04/27 11:36:54  cor134
            % Add nominal (non singular) pose qn
            
            % Copyright (C) 1993-2015, by Peter I. Corke
            %
            % This file is part of The Robotics Toolbox for MATLAB (RTB).
            % 
            % RTB is free software: you can redistribute it and/or modify
            % it under the terms of the GNU Lesser General Public License as published by
            % the Free Software Foundation, either version 3 of the License, or
            % (at your option) any later version.
            % 
            % RTB is distributed in the hope that it will be useful,
            % but WITHOUT ANY WARRANTY; without even the implied warranty of
            % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
            % GNU Lesser General Public License for more details.
            % 
            % You should have received a copy of the GNU Leser General Public License
            % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
            %
            % http://www.petercorke.com
            
            clear L
            deg = pi/180;
            
            % joint angle limits from 
            % A combined optimization method for solving the inverse kinematics problem...
            % Wang & Chen
            % IEEE Trans. RA 7(4) 1991 pp 489-
            L(1) = Link('d', 0, 'a', 0, 'alpha', pi/2, ...
                'I', [0, 0.35, 0, 0, 0, 0], ...
                'r', [0, 0, 0], ...
                'm', 20, ...
                'Jm', 200e-6, ...
                'G', -62.6111, ...
                'B', 1.48e-3, ...
                'Tc', 0, ...
                'qlim', [-160 160]*deg );
            
            L(2) = Link('d', 0, 'a', 0.4318, 'alpha', 0, ...
                'I', [0.13, 0.524, 0.539, 0, 0, 0], ...
                'r', [-0.3638, 0.006, 0.2275], ...
                'm', 17.4, ...
                'Jm', 200e-6, ...
                'G', 107.815, ...
                'B', .817e-3, ...
                'Tc', [0.126 -0.071], ...
                'qlim', [-45 225]*deg );
            
            L(3) = Link('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2,  ...
                'I', [0.066, 0.086, 0.0125, 0, 0, 0], ...
                'r', [-0.0203, -0.0141, 0.070], ...
                'm', 4.8, ...
                'Jm', 200e-6, ...
                'G', -53.7063, ...
                'B', 1.38e-3, ...
                'Tc', [0.132, -0.105], ...
                'qlim', [-225 45]*deg );
            
            L(4) = Link('d', 0.4318, 'a', 0, 'alpha', pi/2,  ...
                'I', [1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0], ...
                'r', [0, 0.019, 0], ...
                'm', 0.82, ...
                'Jm', 33e-6, ...
                'G', 76.0364, ...
                'B', 71.2e-6, ...
                'Tc', [11.2e-3, -16.9e-3], ...
                'qlim', [-110 170]*deg);
            
            L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2,  ...
                'I', [0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0], ...
                'r', [0, 0, 0], ...
                'm', 0.34, ...
                'Jm', 33e-6, ...
                'G', 71.923, ...
                'B', 82.6e-6, ...
                'Tc', [9.26e-3, -14.5e-3], ...
                'qlim', [-100 100]*deg );
            
            
            L(6) = Link('d', 0, 'a', 0, 'alpha', 0,  ...
                'I', [0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0], ...
                'r', [0, 0, 0.032], ...
                'm', 0.09, ...
                'Jm', 33e-6, ...
                'G', 76.686, ...
                'B', 36.7e-6, ...
                'Tc', [3.96e-3, -10.5e-3], ...
                'qlim', [-266 266]*deg );
            
            p560 = SerialLink(L, 'name', 'Puma 560', ...
                'manufacturer', 'Unimation', 'ikine', 'puma', 'comment', 'viscous friction; params of 8/95');
            
            app.robot = p560;
            app.populateListBoxRobot();
            
        end

        % Button pushed function: ExampleStanfordButton
        function ExampleStanfordButtonPushed(app, event)
            %MDL_STANFORD Create model of Stanford arm
            %
            %      mdl_stanford
            %
            % Script creates the workspace variable stanf which describes the 
            % kinematic and dynamic characteristics of the Stanford (Scheinman) arm.
            %
            % Also defines the vectors:
            %   qz   zero joint angle configuration.
            %
            % Note::
            % - SI units are used.
            % - Gear ratios not currently known, though reflected armature inertia 
            %   is known, so gear ratios are set to 1.
            %
            % References::
            % - Kinematic data from "Modelling, Trajectory calculation and Servoing of 
            %   a computer controlled arm".  Stanford AIM-177.  Figure 2.3
            % - Dynamic data from "Robot manipulators: mathematics, programming and control"
            %   Paul 1981, Tables 6.5, 6.6
            % - Dobrotin & Scheinman, "Design of a computer controlled manipulator for
            %   robot research", IJCAI, 1973.
            % 
            % See also SerialLink, mdl_puma560, mdl_puma560akb.
            
            
            % MODEL: Stanford, Stanford Arm, prismatic, 6DOF, standard_DH
            
            % Copyright (C) 1993-2015, by Peter I. Corke
            %
            % This file is part of The Robotics Toolbox for MATLAB (RTB).
            % 
            % RTB is free software: you can redistribute it and/or modify
            % it under the terms of the GNU Lesser General Public License as published by
            % the Free Software Foundation, either version 3 of the License, or
            % (at your option) any later version.
            % 
            % RTB is distributed in the hope that it will be useful,
            % but WITHOUT ANY WARRANTY; without even the implied warranty of
            % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
            % GNU Lesser General Public License for more details.
            % 
            % You should have received a copy of the GNU Leser General Public License
            % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
            %
            % http://www.petercorke.com
            
            clear L
            %             th    d       a    alpha
            L(1) = Link([ 0     0.412   0   -pi/2     0]);
            L(2) = Link([ 0     0.154   0    pi/2     0]);
            L(3) = Link([ -pi/2 0       0    0        1]);  % PRISMATIC link
            L(4) = Link([ 0     0       0   -pi/2     0]);
            L(5) = Link([ 0     0       0    pi/2     0]);
            L(6) = Link([ 0     0.263   0    0        0]);
            
            % guestimates of some parameters
            %
            % According to the IJCAI paper the rack is 38in and the maximum reach is 52in
            % From the image http://infolab.stanford.edu/pub/voy/museum/pictures/display/robots/IMG_2408ArmCenter.JPG
            % and scaled by the rack length (38in) it looks like the minimum stroke is 12in.
            %
            L(3).qlim = [12 12+38] * 0.0254;
            
            % According to the IJCAI paper
            L(1).qlim = [-170 170]*pi/180;
            L(2).qlim = [-170 170]*pi/180;
            L(4).qlim = [-170 170]*pi/180;
            L(5).qlim = [-90 90]*pi/180;
            L(6).qlim = [-170 170]*pi/180;
            
            % Added
            L(3).qlim = [-5, 5];
            
            
            L(1).m = 9.29;
            L(2).m = 5.01;
            L(3).m = 4.25;
            L(4).m = 1.08;
            L(5).m = 0.630;
            L(6).m = 0.51;
            
            L(1).Jm = 0.953;
            L(2).Jm = 2.193;
            L(3).Jm = 0.782;
            L(4).Jm = 0.106;
            L(5).Jm = 0.097;
            L(6).Jm = 0.020;
            
            L(1).G = 1;
            L(2).G = 1;
            L(3).G = 1;
            L(4).G = 1;
            L(5).G = 1;
            L(6).G = 1;
            
            L(1).I = [0.276   0.255   0.071   0   0   0];
            L(2).I = [0.108   0.018   0.100   0   0   0];
            L(3).I = [2.51    2.51    0.006   0   0   0 ];
            L(4).I = [0.002   0.001   0.001   0   0   0 ];
            L(5).I = [0.003   0.0004  0       0   0   0];
            L(6).I = [0.013   0.013   0.0003  0   0   0];
            
            L(1).r = [0    0.0175 -0.1105];
            L(2).r = [0   -1.054  0];
            L(3).r = [0    0      -6.447];
            L(4).r = [0    0.092  -0.054];
            L(5).r = [0    0.566   0.003];
            L(6).r = [0    0       1.554];
            
            qz = [0 0 0 0 0 0];
            
            app.robot = SerialLink(L, 'name', 'Stanford arm');
            app.populateListBoxRobot();
        end

        % Button pushed function: Example2LinkButton
        function Example2LinkButtonPushed(app, event)
            %MDL_TWOLINK Create model of a 2-link mechanism
            %
            % MDL_TWOLINK is a script that creates the workspace variable tl which
            % describes the kinematic and dynamic characteristics of a simple planar
            % 2-link mechanism.
            %
            % Also defines the vector:
            %   qz   corresponds to the zero joint angle configuration.
            %
            % Notes::
            % - SI units are used.
            % - It is a planar mechanism operating in the XY (horizontal) plane and is 
            %   therefore not affected by gravity.
            % - Assume unit length links with all mass (unity) concentrated at the joints.
            %
            % References::
            %  - Based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
            %
            % See also SerialLink, mdl_onelink, mdl_twolink_mdh, mdl_planar2.
            
            % MODEL: generic, planar, dynamics, 2DOF, standard_DH
            
            
            % Copyright (C) 1993-2015, by Peter I. Corke
            %
            % This file is part of The Robotics Toolbox for MATLAB (RTB).
            % 
            % RTB is free software: you can redistribute it and/or modify
            % it under the terms of the GNU Lesser General Public License as published by
            % the Free Software Foundation, either version 3 of the License, or
            % (at your option) any later version.
            % 
            % RTB is distributed in the hope that it will be useful,
            % but WITHOUT ANY WARRANTY; without even the implied warranty of
            % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
            % GNU Lesser General Public License for more details.
            % 
            % You should have received a copy of the GNU Leser General Public License
            % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
            %
            % http://www.petercorke.com
            
            
            a1 = 1;
            a2 = 1;
            %   theta d a alpha
            
            app.robot = SerialLink([
                Link('d', 0, 'a', a1, 'alpha', 0, 'm', 1, 'r', [-0.5 0 0], 'I', [1 1 1], 'B', 0, 'G', 0, 'Jm', 0, 'standard','qlim', [-pi, pi])
                Link('d', 0, 'a', a2, 'alpha', 0, 'm', 1, 'r', [-0.5 0 0], 'I', [1 1 1], 'B', 0, 'G', 0, 'Jm', 0, 'standard','qlim', [-pi, pi])
                ], ...
                'name', 'two link', ...
                'comment', 'from Spong, Hutchinson, Vidyasagar');
            app.populateListBoxRobot();
        end

        % Button pushed function: CalculateDynamicsButton
        function CalculateDynamicsButtonPushed(app, event)
            % Wraps dyno_wrap
            app.print("Starting dynamics...");
            % gets the inputs
            [qInit, qdInit] = app.getDynInitialConditions();
            finalTime = app.FinalTimeEditField.Value;
            jointNum = length(app.robot.links);
            % figures out the torque function
            if app.TypeDropDown.Value == "Constant"
                torqFun = 'constant';
                torqVec = zeros(1,jointNum);
                torqVec = app.torqFunTable.Data;
                for i=1:jointNum
                    
                end
                app.print("Running dynamics with constant torques...");
                [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit, torqVec);
            elseif app.TypeDropDown.Value == "PD"
                torqFun = 'PD';
                for i=1:jointNum

                end
                qStar = app.torqFunTable.Data(1,1:jointNum);
                P = app.torqFunTable.Data(1,jointNum+1);
                D = app.torqFunTable.Data(1,jointNum+2);
                app.print("Running dynamics with PD control...");
                [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit, qStar, P, D);
            elseif app.TypeDropDown.Value == "Custom"
                torqFun = torqFun;
                try
                    app.print("Running dynamics with custom function...")
                    [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit);
                catch ME
                    app.print(ME);
                    return;
                end
            end
            plot(app.axesJointPositions, T, q);
            plot(app.axesJointVelocities, T, qd);

        end

        % Value changed function: listBoxRobot
        function listBoxRobotValueChanged(app, event)
            app.updateGeometryTable();
        end

        % Button pushed function: GenerateButton
        function GenerateButtonPushed(app, event)
            cla(app.axesWorkspace); % clears graph
            gridRes = app.MeshResolutionEditField.Value;
            points = workspaceFinder(app.robot, gridRes);
            set(app.axesWorkspace, 'XGrid', 'on');
            set(app.axesWorkspace, 'YGrid', 'on');
            set(app.axesWorkspace, 'ZGrid', 'on');
            scatter3(app.axesWorkspace, points(:,1), points(:,2), points(:,3));
            
%             if app.PlotStyleDropDown.Value == "Polygon"
%                 %fill3(app.axesWorkspace, points(:,1), points(:,2), points(:,3), 'b')
%                 pointsBnd = boundary(points, 0.8);
%                 if isempty(pointsBnd) % if planar
%                     pointsBnd = boundary(points(:,1:2));
% 
%                     if app.ShellDropDown.Value == "Point Cloud"
%                         scatter3(app.axesWorkspace, points(:,1), points(:,2), points(:,3));
%                     end
%                     tri = delaunayTriangulation(points(pointsBnd,1:2));
%                     bnd = boundaryshape(tri);
%                     plot(app.axesWorkspace, bnd, 'FaceColor','red','FaceAlpha',0.1);
% 
%                 else
%                     tri = delaunayTriangulation(points(pointsBnd,1:2));
%                     [bndCon, bndPts] = freeBoundary(tri);
%                     bnd = points(bndCon,:);
%                     if app.ShellDropDown.Value == "Point Cloud"
%                         scatter3(app.axesWorkspace, points(:,1), points(:,2), points(:,3));
%                     end
%                     plt = fill3(app.axesWorkspace, bnd(:,1), bnd(:,2), bnd(:,3), 'r');
%                     set(plt, 'facealpha', 0.1);
%                     %trisurf(pointsBnd, points(:,1), points(:,2), points(:,3),'Facecolor','red','FaceAlpha',0.1, app.axesWorkspace);
%             
%                 end
%             end
        end

        % Value changed function: ConfigurationNumberEditField
        function ConfigurationNumberEditFieldValueChanged(app, event)
            rowNum = app.ConfigurationNumberEditField.Value;
            colNum = length(app.robot.links);
            app.updateTable(app.kineConfigTable, rowNum, colNum);
        end

        % Button pushed function: AnimateKineButton
        function AnimateKineButtonPushed(app, event)
            % get joint trajectory
            app.print("Animating Forward Kinematics...")
            rowNum = app.ConfigurationNumberEditField.Value;
            if rowNum == 1
                q = app.kineConfigTable.Data(rowNum,:);
                app.safePlot(q);
                return
            else
                q = [];
                numPoints = 10;
                for i = 1:rowNum - 1
                    q0 = app.kineConfigTable.Data(i,:);
                    qf = app.kineConfigTable.Data(i+1,:);
                    [qi, qdi, qddi] = jtraj(q0, qf, numPoints);
                    q = [q; qi];
                end
            end
            
            
            % animate it
            app.safePlot(q);
            
        end

        % Button pushed function: GivePosesButton
        function GivePosesButtonPushed(app, event)
            app.print("Giving End Effector Poses...");
            rowNum = app.ConfigurationNumberEditField.Value;

            q = [];
            for i = 1:rowNum
                q0 = app.kineConfigTable.Data(i,:);
                q = [q; q0];
            end
            
            poses = app.robot.fkine(q); % 4x4xN
            
            pageNum = size(poses,3);
            
            for i=1:pageNum
                posesMat(i:4+i,1:4) = squeeze(poses(1:4,1:4,i));
            end
            
            uitable('Data', posesMat);
        end

        % Value changed function: PoseNumberEditField
        function PoseNumberEditFieldValueChanged(app, event)
            rowNum = app.PoseNumberEditField.Value;
            colNum = 6;
            app.updateTable(app.invKineConfigTable, rowNum, colNum);
        end

        % Value changed function: TypeDropDown
        function TypeDropDownValueChanged(app, event)
            app.updateDynInputTable();  
        end

        % Button pushed function: AnimateDynamicsButton
        function AnimateDynamicsButtonPushed(app, event)
            % Wraps dyno_wrap
            app.print("Starting dynamics...");
            % gets the inputs
            [qInit, qdInit] = app.getDynInitialConditions();
            finalTime = app.FinalTimeEditField.Value;
            jointNum = length(app.robot.links);
            % figures out the torque function
            if app.TypeDropDown.Value == "Constant"
                torqFun = 'constant';
                torqVec = zeros(1,jointNum);
                torqVec = app.torqFunTable.Data;
                for i=1:jointNum
                    
                end
                app.print("Running dynamics with constant torques...");
                [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit, torqVec);
            elseif app.TypeDropDown.Value == "PD"
                torqFun = 'PD';
                for i=1:jointNum

                end
                qStar = app.torqFunTable.Data(1,1:jointNum);
                P = app.torqFunTable.Data(1,jointNum+1);
                D = app.torqFunTable.Data(1,jointNum+2);
                app.print("Running dynamics with PD control...");
                [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit, qStar, P, D);
            elseif app.TypeDropDown.Value == "Custom"
                torqFun = torqFun;
                try
                    app.print("Running dynamics with custom function...")
                    [T, q, qd] = dyno_wrap(app.robot,finalTime, torqFun, qInit, qdInit);
                catch ME
                    app.print(ME);
                    return;
                end
            end
            app.safePlot(q);
            app.robot.teach();
            
        end

        % Button pushed function: AnimateInvKineButton
        function AnimateInvKineButtonPushed(app, event)
            app.print("Animating Inverse Kinematics...")
            rowNum = app.PoseNumberEditField.Value;
            if rowNum == 1
                p = app.invKineConfigTable.Data(rowNum,:);
                tr = rpy2tr(p(1,4:6));
                pos = p(1,1:3);
                tr(1:3,4,:) = pos';
                q = app.robot.ikunc(tr);
                app.safePlot(q);
                return
            else
                q = [];
                numPoints = 10;
                for i = 1:rowNum - 1
                    p0 = app.invKineConfigTable.Data(i,:);
                    pf = app.invKineConfigTable.Data(i+1,:);
                    T0 = rpy2tr(p0(1,4:6));
                    Tf = rpy2tr(pf(1,4:6));
                    pos0 = p0(1,1:3);
                    T0(1:3,4) = pos0';
                    posf = pf(1,1:3);
                    Tf(1:3,4) = posf';
                    q0 = app.robot.ikunc(T0);
                    qf = app.robot.ikunc(Tf);
                    [qi, qdi, qddi] = jtraj(q0, qf, numPoints);
   
                    %q = app.robot.ikunc(tr);
                    q = [q; qi];
                end
            end
            
            
            % animate it
            app.safePlot(q);
        end

        % Button pushed function: AnimateInvKineJacoButton
        function AnimateInvKineJacoButtonPushed(app, event)
            app.print("Animating Inverse Kinematics...")
            rowNum = app.PoseNumberEditField.Value;
            if rowNum == 1
                p = app.invKineConfigTable.Data(rowNum,:);
                q = inv_kin_JacInv(app.robot, p);
                app.safePlot(q);
                return
            else
                q = [];
                numPoints = 10;
                for i = 1:rowNum - 1
                    p0 = app.invKineConfigTable.Data(i,:);
                    pf = app.invKineConfigTable.Data(i+1,:);
                   
%                     q0 = inv_kin_JacInv(app.robot, p0);
%                     qf = inv_kin_JacInv(app.robot, pf);
                    q0 = inv_kin_JacInv_V2(app.robot, p0);
                    qf = inv_kin_JacInv_V2(app.robot, pf);
                    [qi, qdi, qddi] = jtraj(q0, qf, numPoints);
   
                    %q = app.robot.ikunc(tr);
                    q = [q; qi];
                end
            end
            
            
            % animate it
            app.safePlot(q);
        end

        % Callback function
        function diffAnimateRobotButtonPushed(app, event)
            % get joint trajectory
            app.print("Animating Differential Forward Kinematics...")
            rowNum = app.diffConfigNumberEditField.Value;
            if rowNum == 1
                q = app.diffKineConfigTable.Data(rowNum,:);
                app.safePlot(q);
                return
            else
                q = [];
                numPoints = 10;
                for i = 1:rowNum - 1
                    q0 = app.diffKineConfigTable.Data(i,:);
                    qf = app.diffKineConfigTable.Data(i+1,:);
                    [qi, qdi, qddi] = jtraj(q0, qf, numPoints);
                    q = [q; qi];
                end
            end
            
            
            % animate it
            app.safePlot(q);
        end

        % Display data changed function: tranfTable
        function tranfTableDisplayDataChanged(app, event)
            newDisplayData = app.tranfTable.DisplayData;
        end

        % Cell edit callback: tranfTable
        function tranfTableCellEdit(app, event)
            % plot the transformation between orgin and this new one
            app.updateCoordinateTransform('T');
            
        end

        % Cell edit callback: converterTable
        function converterTableCellEdit(app, event)
            app.updateCoordinateTransform('Convert');
            
        end

        % Button pushed function: ClearButton
        function ClearButtonPushed(app, event)
            cla(app.axesTranf);
            app.converterTable.Data = zeros(1,length(app.converterTable.Data));
            app.tranfTable.Data = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        end

        % Value changed function: RzRotationSlider
        function RzRotationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
            
        end

        % Value changed function: RyRotationSlider
        function RyRotationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
        end

        % Value changed function: RxRotationSlider
        function RxRotationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
            
        end

        % Value changed function: xTranslationSlider
        function xTranslationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
            
        end

        % Value changed function: yTranslationSlider
        function yTranslationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
            
        end

        % Value changed function: zTranslationSlider
        function zTranslationSliderValueChanged(app, event)
            app.updateCoordinateTransform('Sliders');
            
        end

        % Button pushed function: GiveConfigurationsButton
        function GiveConfigurationsButtonPushed(app, event)
            
        end

        % Selection change function: TabGroup
        function TabGroupSelectionChanged(app, event)
            selectedTab = app.TabGroup.SelectedTab;
            if strcmp(selectedTab.Title, 'Equations of Motion')
                app.populateEOM();
                app.updateEOM();
                
            end
        end

        % Button pushed function: DisplaySelectedMatrixButton
        function DisplaySelectedMatrixButtonPushed(app, event)
            app.updateEOM();
        end

        % Button pushed function: FindSingularitiesButton
        function FindSingularitiesButtonPushed(app, event)
            [singleSingulars, doubleSingulars] = singFinder(app.robot);
            % singleSingulars % unknown n by 2
            % doubleSingulars % unknown m by 4
            
            if size(singleSingulars, 1) >= size(doubleSingulars, 1)
                rowNum = size(singleSingulars, 1);
            else
                rowNum = size(doubleSingulars, 1);
            end
            
            colNum = 6;
            
            app.updateTable(app.singularTable, rowNum, colNum);
            
            app.singularTable.Data(1:size(singleSingulars, 1), 1:2) = singleSingulars;
            app.singularTable.Data(1:(size(doubleSingulars, 1)), 3:6) = doubleSingulars;
            
            
        end

        % Button pushed function: ShowConfigurationsButton
        function ShowConfigurationsButtonPushed(app, event)
            app.print("Animating Inverse Kinematics...")
            rowNum = app.PoseNumberEditField.Value;
            q = [];
            for i = 1:rowNum
                p = app.invKineConfigTable.Data(i,:);
                tr = rpy2tr(p(1,4:6));
                pos = p(1,1:3);
                tr(1:3,4,:) = pos';
                q0 = app.robot.ikunc(tr);
                q = [q; q0];
            end
               app.safePlot(q, 'delay', 2);
                return
            
            
        end

        % Button pushed function: ShowPosesButton
        function ShowPosesButtonPushed(app, event)
            rowNum = size(app.kineConfigTable.Data,1);
            q = app.kineConfigTable.Data(1:rowNum,:);
            app.safePlot(q, 'delay', 2);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'UI Figure';

            % Create File
            app.File = uimenu(app.UIFigure);
            app.File.Text = 'File';

            % Create tabGroupMain
            app.tabGroupMain = uitabgroup(app.UIFigure);
            app.tabGroupMain.SelectionChangedFcn = createCallbackFcn(app, @tabGroupMainSelectionChanged, true);
            app.tabGroupMain.Position = [1 64 640 417];

            % Create tabRobot
            app.tabRobot = uitab(app.tabGroupMain);
            app.tabRobot.Tooltip = {'Define your masterful robot here!'};
            app.tabRobot.Title = 'Robot';

            % Create panelRobot
            app.panelRobot = uipanel(app.tabRobot);
            app.panelRobot.Title = 'Robot Options';
            app.panelRobot.Scrollable = 'on';
            app.panelRobot.Position = [150 290 489 102];

            % Create AddNewLinkButton
            app.AddNewLinkButton = uibutton(app.panelRobot, 'push');
            app.AddNewLinkButton.ButtonPushedFcn = createCallbackFcn(app, @AddNewLinkButtonPushed, true);
            app.AddNewLinkButton.Position = [12 50 100 22];
            app.AddNewLinkButton.Text = 'Add New Link';

            % Create RemoveLinkButton
            app.RemoveLinkButton = uibutton(app.panelRobot, 'push');
            app.RemoveLinkButton.ButtonPushedFcn = createCallbackFcn(app, @RemoveLinkButtonPushed, true);
            app.RemoveLinkButton.Position = [12 15 100 22];
            app.RemoveLinkButton.Text = 'Remove Link';

            % Create UpdateLinkButton
            app.UpdateLinkButton = uibutton(app.panelRobot, 'push');
            app.UpdateLinkButton.ButtonPushedFcn = createCallbackFcn(app, @UpdateLinkButtonPushed, true);
            app.UpdateLinkButton.Position = [128 50 100 22];
            app.UpdateLinkButton.Text = {'Update Link'; ''};

            % Create VisualzeRobotButton
            app.VisualzeRobotButton = uibutton(app.panelRobot, 'push');
            app.VisualzeRobotButton.ButtonPushedFcn = createCallbackFcn(app, @VisualzeRobotButtonPushed, true);
            app.VisualzeRobotButton.Position = [247 50 100 22];
            app.VisualzeRobotButton.Text = {'Visualze Robot'; ''};

            % Create EditFullRobotButton
            app.EditFullRobotButton = uibutton(app.panelRobot, 'push');
            app.EditFullRobotButton.ButtonPushedFcn = createCallbackFcn(app, @EditFullRobotButtonPushed, true);
            app.EditFullRobotButton.Position = [128 15 100 22];
            app.EditFullRobotButton.Text = {'Edit Full Robot'; ''};

            % Create ExamplePumaButton
            app.ExamplePumaButton = uibutton(app.panelRobot, 'push');
            app.ExamplePumaButton.ButtonPushedFcn = createCallbackFcn(app, @ExamplePumaButtonPushed, true);
            app.ExamplePumaButton.Position = [359 15 111 22];
            app.ExamplePumaButton.Text = {'Example Puma'; ''};

            % Create ExampleStanfordButton
            app.ExampleStanfordButton = uibutton(app.panelRobot, 'push');
            app.ExampleStanfordButton.ButtonPushedFcn = createCallbackFcn(app, @ExampleStanfordButtonPushed, true);
            app.ExampleStanfordButton.Position = [359 50 111 22];
            app.ExampleStanfordButton.Text = 'Example Stanford';

            % Create Example2LinkButton
            app.Example2LinkButton = uibutton(app.panelRobot, 'push');
            app.Example2LinkButton.ButtonPushedFcn = createCallbackFcn(app, @Example2LinkButtonPushed, true);
            app.Example2LinkButton.Position = [247 15 100 22];
            app.Example2LinkButton.Text = 'Example 2-Link';

            % Create panelLink
            app.panelLink = uipanel(app.tabRobot);
            app.panelLink.Title = 'Link Descritpion';
            app.panelLink.Scrollable = 'on';
            app.panelLink.Position = [151 2 487 289];

            % Create tabGroupLinkDef
            app.tabGroupLinkDef = uitabgroup(app.panelLink);
            app.tabGroupLinkDef.Position = [0 7 486 260];

            % Create tabLinkKine
            app.tabLinkKine = uitab(app.tabGroupLinkDef);
            app.tabLinkKine.Title = 'Kinematics';

            % Create panelLinkDefOptions
            app.panelLinkDefOptions = uipanel(app.tabLinkKine);
            app.panelLinkDefOptions.Title = 'Definition Options';
            app.panelLinkDefOptions.Position = [0 150 490 85];

            % Create GridLayout
            app.GridLayout = uigridlayout(app.panelLinkDefOptions);

            % Create GeometryLabel
            app.GeometryLabel = uilabel(app.GridLayout);
            app.GeometryLabel.HorizontalAlignment = 'center';
            app.GeometryLabel.Layout.Row = 2;
            app.GeometryLabel.Layout.Column = 1;
            app.GeometryLabel.Text = 'Geometry:';

            % Create dropDownGeometry
            app.dropDownGeometry = uidropdown(app.GridLayout);
            app.dropDownGeometry.Items = {'DH Parameters', 'Geometric Description', ''};
            app.dropDownGeometry.ValueChangedFcn = createCallbackFcn(app, @dropDownGeometryValueChanged, true);
            app.dropDownGeometry.Layout.Row = 2;
            app.dropDownGeometry.Layout.Column = 2;
            app.dropDownGeometry.Value = 'DH Parameters';

            % Create JointTypeDropDownLabel
            app.JointTypeDropDownLabel = uilabel(app.GridLayout);
            app.JointTypeDropDownLabel.HorizontalAlignment = 'center';
            app.JointTypeDropDownLabel.Layout.Row = 1;
            app.JointTypeDropDownLabel.Layout.Column = 1;
            app.JointTypeDropDownLabel.Text = 'Joint Type:';

            % Create dropDownJointType
            app.dropDownJointType = uidropdown(app.GridLayout);
            app.dropDownJointType.Items = {'Revolute', 'Prismatic', ''};
            app.dropDownJointType.ValueChangedFcn = createCallbackFcn(app, @dropDownJointTypeValueChanged, true);
            app.dropDownJointType.Layout.Row = 1;
            app.dropDownJointType.Layout.Column = 2;
            app.dropDownJointType.Value = 'Revolute';

            % Create panelGeometry
            app.panelGeometry = uipanel(app.tabLinkKine);
            app.panelGeometry.Title = 'Link Geometry';
            app.panelGeometry.Position = [0 -7 490 158];

            % Create GridLayout2
            app.GridLayout2 = uigridlayout(app.panelGeometry);
            app.GridLayout2.ColumnWidth = {'1x', '0.5x'};
            app.GridLayout2.RowHeight = {20, 20, 20, 20, 20, 20};
            app.GridLayout2.Scrollable = 'on';

            % Create EditFieldLabel
            app.EditFieldLabel = uilabel(app.GridLayout2);
            app.EditFieldLabel.HorizontalAlignment = 'right';
            app.EditFieldLabel.Layout.Row = 1;
            app.EditFieldLabel.Layout.Column = 1;
            app.EditFieldLabel.Text = 'Edit Field';

            % Create fieldKineDOrZDir
            app.fieldKineDOrZDir = uieditfield(app.GridLayout2, 'numeric');
            app.fieldKineDOrZDir.Layout.Row = 1;
            app.fieldKineDOrZDir.Layout.Column = 2;

            % Create EditField2Label
            app.EditField2Label = uilabel(app.GridLayout2);
            app.EditField2Label.HorizontalAlignment = 'right';
            app.EditField2Label.Layout.Row = 2;
            app.EditField2Label.Layout.Column = 1;
            app.EditField2Label.Text = 'Edit Field2';

            % Create fieldKineThetaOrLength
            app.fieldKineThetaOrLength = uieditfield(app.GridLayout2, 'numeric');
            app.fieldKineThetaOrLength.Layout.Row = 2;
            app.fieldKineThetaOrLength.Layout.Column = 2;

            % Create EditField3Label
            app.EditField3Label = uilabel(app.GridLayout2);
            app.EditField3Label.HorizontalAlignment = 'right';
            app.EditField3Label.Layout.Row = 3;
            app.EditField3Label.Layout.Column = 1;
            app.EditField3Label.Text = 'Edit Field3';

            % Create fieldKineAOrVertOffset
            app.fieldKineAOrVertOffset = uieditfield(app.GridLayout2, 'numeric');
            app.fieldKineAOrVertOffset.Layout.Row = 3;
            app.fieldKineAOrVertOffset.Layout.Column = 2;

            % Create EditField4Label
            app.EditField4Label = uilabel(app.GridLayout2);
            app.EditField4Label.HorizontalAlignment = 'right';
            app.EditField4Label.Layout.Row = 4;
            app.EditField4Label.Layout.Column = 1;
            app.EditField4Label.Text = 'Edit Field4';

            % Create fieldKineAlphaOrAngOffset
            app.fieldKineAlphaOrAngOffset = uieditfield(app.GridLayout2, 'numeric');
            app.fieldKineAlphaOrAngOffset.Layout.Row = 4;
            app.fieldKineAlphaOrAngOffset.Layout.Column = 2;

            % Create JointMinimimumEditFieldLabel
            app.JointMinimimumEditFieldLabel = uilabel(app.GridLayout2);
            app.JointMinimimumEditFieldLabel.HorizontalAlignment = 'right';
            app.JointMinimimumEditFieldLabel.Layout.Row = 5;
            app.JointMinimimumEditFieldLabel.Layout.Column = 1;
            app.JointMinimimumEditFieldLabel.Text = 'Joint Minimimum';

            % Create fieldJointMin
            app.fieldJointMin = uieditfield(app.GridLayout2, 'numeric');
            app.fieldJointMin.Layout.Row = 5;
            app.fieldJointMin.Layout.Column = 2;

            % Create JointMaximumEditFieldLabel
            app.JointMaximumEditFieldLabel = uilabel(app.GridLayout2);
            app.JointMaximumEditFieldLabel.HorizontalAlignment = 'right';
            app.JointMaximumEditFieldLabel.Layout.Row = 6;
            app.JointMaximumEditFieldLabel.Layout.Column = 1;
            app.JointMaximumEditFieldLabel.Text = {'Joint Maximum'; ''};

            % Create fieldJointMax
            app.fieldJointMax = uieditfield(app.GridLayout2, 'numeric');
            app.fieldJointMax.Layout.Row = 6;
            app.fieldJointMax.Layout.Column = 2;

            % Create tabLinkDynamics
            app.tabLinkDynamics = uitab(app.tabGroupLinkDef);
            app.tabLinkDynamics.Title = 'Dynamics';

            % Create GridLayout3
            app.GridLayout3 = uigridlayout(app.tabLinkDynamics);
            app.GridLayout3.RowHeight = {20, 20, 80, 20, 20, 20, 20};
            app.GridLayout3.Scrollable = 'on';

            % Create GridCOGPos
            app.GridCOGPos = uigridlayout(app.GridLayout3);
            app.GridCOGPos.ColumnWidth = {'1x', '1x', '1x'};
            app.GridCOGPos.RowHeight = {'1x'};
            app.GridCOGPos.Padding = [0 0 0 0];
            app.GridCOGPos.Layout.Row = 2;
            app.GridCOGPos.Layout.Column = 2;

            % Create COG1
            app.COG1 = uieditfield(app.GridCOGPos, 'numeric');
            app.COG1.Layout.Row = 1;
            app.COG1.Layout.Column = 1;

            % Create COG2
            app.COG2 = uieditfield(app.GridCOGPos, 'numeric');
            app.COG2.Layout.Row = 1;
            app.COG2.Layout.Column = 2;

            % Create COG3
            app.COG3 = uieditfield(app.GridCOGPos, 'numeric');
            app.COG3.Layout.Row = 1;
            app.COG3.Layout.Column = 3;

            % Create GridInertiaMatrix
            app.GridInertiaMatrix = uigridlayout(app.GridLayout3);
            app.GridInertiaMatrix.ColumnWidth = {'1x', '1x', '1x'};
            app.GridInertiaMatrix.RowHeight = {'1x', '1x', '1x'};
            app.GridInertiaMatrix.Padding = [0 0 0 0];
            app.GridInertiaMatrix.Layout.Row = 3;
            app.GridInertiaMatrix.Layout.Column = 2;

            % Create Inertia11
            app.Inertia11 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia11.Layout.Row = 1;
            app.Inertia11.Layout.Column = 1;
            app.Inertia11.Value = 1;

            % Create Inertia12
            app.Inertia12 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia12.Layout.Row = 1;
            app.Inertia12.Layout.Column = 2;

            % Create Inertia13
            app.Inertia13 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia13.Layout.Row = 1;
            app.Inertia13.Layout.Column = 3;

            % Create Inertia21
            app.Inertia21 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia21.Layout.Row = 2;
            app.Inertia21.Layout.Column = 1;

            % Create Inertia31
            app.Inertia31 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia31.Layout.Row = 3;
            app.Inertia31.Layout.Column = 1;

            % Create Inertia22
            app.Inertia22 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia22.Layout.Row = 2;
            app.Inertia22.Layout.Column = 2;
            app.Inertia22.Value = 1;

            % Create Inertia32
            app.Inertia32 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia32.Layout.Row = 3;
            app.Inertia32.Layout.Column = 2;

            % Create Inertia23
            app.Inertia23 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia23.Layout.Row = 2;
            app.Inertia23.Layout.Column = 3;

            % Create Inertia33
            app.Inertia33 = uieditfield(app.GridInertiaMatrix, 'numeric');
            app.Inertia33.Layout.Row = 3;
            app.Inertia33.Layout.Column = 3;
            app.Inertia33.Value = 1;

            % Create labelCenterofGravityLocation
            app.labelCenterofGravityLocation = uilabel(app.GridLayout3);
            app.labelCenterofGravityLocation.Layout.Row = 2;
            app.labelCenterofGravityLocation.Layout.Column = 1;
            app.labelCenterofGravityLocation.Text = {'Center of Gravity Location'; ''};

            % Create labelInertiaMatrixaboutCenterofGravity
            app.labelInertiaMatrixaboutCenterofGravity = uilabel(app.GridLayout3);
            app.labelInertiaMatrixaboutCenterofGravity.Layout.Row = 3;
            app.labelInertiaMatrixaboutCenterofGravity.Layout.Column = 1;
            app.labelInertiaMatrixaboutCenterofGravity.Text = 'Inertia Matrix about Center of Gravity';

            % Create LinkMassEditFieldLabel
            app.LinkMassEditFieldLabel = uilabel(app.GridLayout3);
            app.LinkMassEditFieldLabel.Layout.Row = 1;
            app.LinkMassEditFieldLabel.Layout.Column = 1;
            app.LinkMassEditFieldLabel.Text = {'Link Mass'; ''};

            % Create fieldDynLinkMass
            app.fieldDynLinkMass = uieditfield(app.GridLayout3, 'numeric');
            app.fieldDynLinkMass.Layout.Row = 1;
            app.fieldDynLinkMass.Layout.Column = 2;
            app.fieldDynLinkMass.Value = 1;

            % Create ViscousFrictionEditFieldLabel
            app.ViscousFrictionEditFieldLabel = uilabel(app.GridLayout3);
            app.ViscousFrictionEditFieldLabel.Layout.Row = 4;
            app.ViscousFrictionEditFieldLabel.Layout.Column = 1;
            app.ViscousFrictionEditFieldLabel.Text = {'Viscous Friction'; ''};

            % Create fieldDynViscousFriction
            app.fieldDynViscousFriction = uieditfield(app.GridLayout3, 'numeric');
            app.fieldDynViscousFriction.Layout.Row = 4;
            app.fieldDynViscousFriction.Layout.Column = 2;
            app.fieldDynViscousFriction.Value = 0.1;

            % Create CoulombFrictionEditFieldLabel
            app.CoulombFrictionEditFieldLabel = uilabel(app.GridLayout3);
            app.CoulombFrictionEditFieldLabel.Layout.Row = 5;
            app.CoulombFrictionEditFieldLabel.Layout.Column = 1;
            app.CoulombFrictionEditFieldLabel.Text = {'Coulomb Friction'; ''};

            % Create fieldDynCoulombFriction
            app.fieldDynCoulombFriction = uieditfield(app.GridLayout3, 'numeric');
            app.fieldDynCoulombFriction.Layout.Row = 5;
            app.fieldDynCoulombFriction.Layout.Column = 2;

            % Create GearRatioEditFieldLabel
            app.GearRatioEditFieldLabel = uilabel(app.GridLayout3);
            app.GearRatioEditFieldLabel.Layout.Row = 6;
            app.GearRatioEditFieldLabel.Layout.Column = 1;
            app.GearRatioEditFieldLabel.Text = 'Gear Ratio';

            % Create fieldDynGearRatio
            app.fieldDynGearRatio = uieditfield(app.GridLayout3, 'numeric');
            app.fieldDynGearRatio.Layout.Row = 6;
            app.fieldDynGearRatio.Layout.Column = 2;
            app.fieldDynGearRatio.Value = 1;

            % Create MotorInertiaEditFieldLabel
            app.MotorInertiaEditFieldLabel = uilabel(app.GridLayout3);
            app.MotorInertiaEditFieldLabel.Layout.Row = 7;
            app.MotorInertiaEditFieldLabel.Layout.Column = 1;
            app.MotorInertiaEditFieldLabel.Text = {'Motor Inertia'; ''; ''};

            % Create fieldDynMotorInertia
            app.fieldDynMotorInertia = uieditfield(app.GridLayout3, 'numeric');
            app.fieldDynMotorInertia.Layout.Row = 7;
            app.fieldDynMotorInertia.Layout.Column = 2;
            app.fieldDynMotorInertia.Value = 0.1;

            % Create listBoxRobot
            app.listBoxRobot = uilistbox(app.tabRobot);
            app.listBoxRobot.Items = {};
            app.listBoxRobot.ValueChangedFcn = createCallbackFcn(app, @listBoxRobotValueChanged, true);
            app.listBoxRobot.Position = [0 0 151 392];
            app.listBoxRobot.Value = {};

            % Create CoordinateTransformTab
            app.CoordinateTransformTab = uitab(app.tabGroupMain);
            app.CoordinateTransformTab.Title = 'Coordinate Transform';

            % Create GridLayout23
            app.GridLayout23 = uigridlayout(app.CoordinateTransformTab);
            app.GridLayout23.RowHeight = {'1x'};

            % Create axesTranf
            app.axesTranf = uiaxes(app.GridLayout23);
            title(app.axesTranf, 'Transformation Visualiztion')
            xlabel(app.axesTranf, 'X')
            ylabel(app.axesTranf, 'Y')
            app.axesTranf.PlotBoxAspectRatio = [1 1.23125 1];
            app.axesTranf.Layout.Row = 1;
            app.axesTranf.Layout.Column = 2;

            % Create GridLayout24
            app.GridLayout24 = uigridlayout(app.GridLayout23);
            app.GridLayout24.ColumnWidth = {'1x'};
            app.GridLayout24.Padding = [0 0 0 0];
            app.GridLayout24.Layout.Row = 1;
            app.GridLayout24.Layout.Column = 1;

            % Create HomogenousTransformationPanel
            app.HomogenousTransformationPanel = uipanel(app.GridLayout24);
            app.HomogenousTransformationPanel.Title = 'Homogenous Transformation';
            app.HomogenousTransformationPanel.Layout.Row = 1;
            app.HomogenousTransformationPanel.Layout.Column = 1;

            % Create GridLayout25
            app.GridLayout25 = uigridlayout(app.HomogenousTransformationPanel);
            app.GridLayout25.ColumnWidth = {'1x'};
            app.GridLayout25.RowHeight = {'1x'};

            % Create tranfTable
            app.tranfTable = uitable(app.GridLayout25);
            app.tranfTable.ColumnName = {''};
            app.tranfTable.RowName = {};
            app.tranfTable.ColumnEditable = true;
            app.tranfTable.CellEditCallback = createCallbackFcn(app, @tranfTableCellEdit, true);
            app.tranfTable.DisplayDataChangedFcn = createCallbackFcn(app, @tranfTableDisplayDataChanged, true);
            app.tranfTable.Layout.Row = 1;
            app.tranfTable.Layout.Column = 1;

            % Create Panel_2
            app.Panel_2 = uipanel(app.GridLayout24);
            app.Panel_2.Layout.Row = 2;
            app.Panel_2.Layout.Column = 1;

            % Create GridLayout26
            app.GridLayout26 = uigridlayout(app.Panel_2);
            app.GridLayout26.ColumnWidth = {'1x'};
            app.GridLayout26.RowHeight = {'1x'};

            % Create TabGroup4
            app.TabGroup4 = uitabgroup(app.GridLayout26);
            app.TabGroup4.Layout.Row = 1;
            app.TabGroup4.Layout.Column = 1;

            % Create ConvertTab
            app.ConvertTab = uitab(app.TabGroup4);
            app.ConvertTab.Title = 'Convert';

            % Create GridLayout27
            app.GridLayout27 = uigridlayout(app.ConvertTab);
            app.GridLayout27.ColumnWidth = {'1x'};
            app.GridLayout27.RowHeight = {'0.4x', '0.4x', '1x'};

            % Create converterTable
            app.converterTable = uitable(app.GridLayout27);
            app.converterTable.ColumnName = {'Column 1'; 'Column 2'; 'Column 3'; 'Column 4'};
            app.converterTable.RowName = {};
            app.converterTable.ColumnEditable = true;
            app.converterTable.CellEditCallback = createCallbackFcn(app, @converterTableCellEdit, true);
            app.converterTable.Layout.Row = 3;
            app.converterTable.Layout.Column = 1;

            % Create GridLayout28
            app.GridLayout28 = uigridlayout(app.GridLayout27);
            app.GridLayout28.RowHeight = {'1x'};
            app.GridLayout28.Padding = [0 0 0 0];
            app.GridLayout28.Layout.Row = 1;
            app.GridLayout28.Layout.Column = 1;

            % Create ConverttoTransformLabel
            app.ConverttoTransformLabel = uilabel(app.GridLayout28);
            app.ConverttoTransformLabel.HorizontalAlignment = 'right';
            app.ConverttoTransformLabel.Layout.Row = 1;
            app.ConverttoTransformLabel.Layout.Column = 1;
            app.ConverttoTransformLabel.Text = 'Convert to Transform';

            % Create ConvertToTDropDown
            app.ConvertToTDropDown = uidropdown(app.GridLayout28);
            app.ConvertToTDropDown.Items = {'RPY', 'ZYZ', 'DH'};
            app.ConvertToTDropDown.Layout.Row = 1;
            app.ConvertToTDropDown.Layout.Column = 2;
            app.ConvertToTDropDown.Value = 'RPY';

            % Create GridLayout29
            app.GridLayout29 = uigridlayout(app.GridLayout27);
            app.GridLayout29.RowHeight = {'1x'};
            app.GridLayout29.Padding = [0 0 0 0];
            app.GridLayout29.Layout.Row = 2;
            app.GridLayout29.Layout.Column = 1;

            % Create ClearButton
            app.ClearButton = uibutton(app.GridLayout29, 'push');
            app.ClearButton.ButtonPushedFcn = createCallbackFcn(app, @ClearButtonPushed, true);
            app.ClearButton.Layout.Row = 1;
            app.ClearButton.Layout.Column = 2;
            app.ClearButton.Text = 'Clear';

            % Create VisualizeSlideTab
            app.VisualizeSlideTab = uitab(app.TabGroup4);
            app.VisualizeSlideTab.Title = 'Sliders';

            % Create GridLayout30
            app.GridLayout30 = uigridlayout(app.VisualizeSlideTab);
            app.GridLayout30.RowHeight = {'1x', '1x', '1x', '1x', '1x', '1x'};

            % Create RzRotationSliderLabel
            app.RzRotationSliderLabel = uilabel(app.GridLayout30);
            app.RzRotationSliderLabel.HorizontalAlignment = 'right';
            app.RzRotationSliderLabel.Layout.Row = 1;
            app.RzRotationSliderLabel.Layout.Column = 1;
            app.RzRotationSliderLabel.Text = 'Rz Rotation';

            % Create RzRotationSlider
            app.RzRotationSlider = uislider(app.GridLayout30);
            app.RzRotationSlider.Limits = [-3.14159265358979 3.14159265358979];
            app.RzRotationSlider.ValueChangedFcn = createCallbackFcn(app, @RzRotationSliderValueChanged, true);
            app.RzRotationSlider.Layout.Row = 1;
            app.RzRotationSlider.Layout.Column = 2;

            % Create RyRotationSliderLabel
            app.RyRotationSliderLabel = uilabel(app.GridLayout30);
            app.RyRotationSliderLabel.HorizontalAlignment = 'right';
            app.RyRotationSliderLabel.Layout.Row = 2;
            app.RyRotationSliderLabel.Layout.Column = 1;
            app.RyRotationSliderLabel.Text = {'Ry Rotation'; ''};

            % Create RyRotationSlider
            app.RyRotationSlider = uislider(app.GridLayout30);
            app.RyRotationSlider.Limits = [-3.14159265358979 3.14159265358979];
            app.RyRotationSlider.ValueChangedFcn = createCallbackFcn(app, @RyRotationSliderValueChanged, true);
            app.RyRotationSlider.Layout.Row = 2;
            app.RyRotationSlider.Layout.Column = 2;

            % Create RxRotationSliderLabel
            app.RxRotationSliderLabel = uilabel(app.GridLayout30);
            app.RxRotationSliderLabel.HorizontalAlignment = 'right';
            app.RxRotationSliderLabel.Layout.Row = 3;
            app.RxRotationSliderLabel.Layout.Column = 1;
            app.RxRotationSliderLabel.Text = 'Rx Rotation';

            % Create RxRotationSlider
            app.RxRotationSlider = uislider(app.GridLayout30);
            app.RxRotationSlider.Limits = [-3.14159265358979 3.14159265358979];
            app.RxRotationSlider.ValueChangedFcn = createCallbackFcn(app, @RxRotationSliderValueChanged, true);
            app.RxRotationSlider.Layout.Row = 3;
            app.RxRotationSlider.Layout.Column = 2;

            % Create xTranslationSliderLabel
            app.xTranslationSliderLabel = uilabel(app.GridLayout30);
            app.xTranslationSliderLabel.HorizontalAlignment = 'right';
            app.xTranslationSliderLabel.Layout.Row = 4;
            app.xTranslationSliderLabel.Layout.Column = 1;
            app.xTranslationSliderLabel.Text = 'x Translation';

            % Create xTranslationSlider
            app.xTranslationSlider = uislider(app.GridLayout30);
            app.xTranslationSlider.Limits = [-5 5];
            app.xTranslationSlider.ValueChangedFcn = createCallbackFcn(app, @xTranslationSliderValueChanged, true);
            app.xTranslationSlider.Layout.Row = 4;
            app.xTranslationSlider.Layout.Column = 2;

            % Create yTranslationSliderLabel
            app.yTranslationSliderLabel = uilabel(app.GridLayout30);
            app.yTranslationSliderLabel.HorizontalAlignment = 'right';
            app.yTranslationSliderLabel.Layout.Row = 5;
            app.yTranslationSliderLabel.Layout.Column = 1;
            app.yTranslationSliderLabel.Text = 'y Translation';

            % Create yTranslationSlider
            app.yTranslationSlider = uislider(app.GridLayout30);
            app.yTranslationSlider.Limits = [-5 5];
            app.yTranslationSlider.ValueChangedFcn = createCallbackFcn(app, @yTranslationSliderValueChanged, true);
            app.yTranslationSlider.Layout.Row = 5;
            app.yTranslationSlider.Layout.Column = 2;

            % Create zTranslationSliderLabel
            app.zTranslationSliderLabel = uilabel(app.GridLayout30);
            app.zTranslationSliderLabel.HorizontalAlignment = 'right';
            app.zTranslationSliderLabel.Layout.Row = 6;
            app.zTranslationSliderLabel.Layout.Column = 1;
            app.zTranslationSliderLabel.Text = 'z Translation';

            % Create zTranslationSlider
            app.zTranslationSlider = uislider(app.GridLayout30);
            app.zTranslationSlider.Limits = [-5 5];
            app.zTranslationSlider.ValueChangedFcn = createCallbackFcn(app, @zTranslationSliderValueChanged, true);
            app.zTranslationSlider.Layout.Row = 6;
            app.zTranslationSlider.Layout.Column = 2;

            % Create WorkspaceTab
            app.WorkspaceTab = uitab(app.tabGroupMain);
            app.WorkspaceTab.Title = 'Workspace';

            % Create WorkspaceOptionsPanel
            app.WorkspaceOptionsPanel = uipanel(app.WorkspaceTab);
            app.WorkspaceOptionsPanel.Title = 'Workspace Options';
            app.WorkspaceOptionsPanel.Position = [1 220 260 172];

            % Create GridLayout4
            app.GridLayout4 = uigridlayout(app.WorkspaceOptionsPanel);

            % Create GenerateButton
            app.GenerateButton = uibutton(app.GridLayout4, 'push');
            app.GenerateButton.ButtonPushedFcn = createCallbackFcn(app, @GenerateButtonPushed, true);
            app.GenerateButton.Layout.Row = 2;
            app.GenerateButton.Layout.Column = 2;
            app.GenerateButton.Text = 'Generate';

            % Create MeshResolutionEditFieldLabel
            app.MeshResolutionEditFieldLabel = uilabel(app.GridLayout4);
            app.MeshResolutionEditFieldLabel.HorizontalAlignment = 'center';
            app.MeshResolutionEditFieldLabel.Layout.Row = 1;
            app.MeshResolutionEditFieldLabel.Layout.Column = 1;
            app.MeshResolutionEditFieldLabel.Text = 'Mesh Resolution';

            % Create MeshResolutionEditField
            app.MeshResolutionEditField = uieditfield(app.GridLayout4, 'numeric');
            app.MeshResolutionEditField.Limits = [0 100];
            app.MeshResolutionEditField.RoundFractionalValues = 'on';
            app.MeshResolutionEditField.Layout.Row = 1;
            app.MeshResolutionEditField.Layout.Column = 2;
            app.MeshResolutionEditField.Value = 5;

            % Create VisualiziationPanel
            app.VisualiziationPanel = uipanel(app.WorkspaceTab);
            app.VisualiziationPanel.Title = 'Visualiziation';
            app.VisualiziationPanel.Position = [260 -2 380 394];

            % Create axesWorkspace
            app.axesWorkspace = uiaxes(app.VisualiziationPanel);
            title(app.axesWorkspace, 'Work Space Visualization')
            xlabel(app.axesWorkspace, 'X')
            ylabel(app.axesWorkspace, 'Y')
            app.axesWorkspace.PlotBoxAspectRatio = [1.06089743589744 1 1];
            app.axesWorkspace.Position = [1 3 378 367];

            % Create TextArea
            app.TextArea = uitextarea(app.WorkspaceTab);
            app.TextArea.Editable = 'off';
            app.TextArea.Position = [12 12 238 195];
            app.TextArea.Value = {'Warning! For robots with large number of joints (n>4) finding workspace can be very resouce intensive. Keep mesh resolution low to avoid issues.'; ''};

            % Create KinematicsTab
            app.KinematicsTab = uitab(app.tabGroupMain);
            app.KinematicsTab.Title = 'Kinematics';

            % Create TabGroup2
            app.TabGroup2 = uitabgroup(app.KinematicsTab);
            app.TabGroup2.Position = [0 -2 639 394];

            % Create ForwardKinematicsTab
            app.ForwardKinematicsTab = uitab(app.TabGroup2);
            app.ForwardKinematicsTab.Title = 'Forward Kinematics';

            % Create GridLayout19
            app.GridLayout19 = uigridlayout(app.ForwardKinematicsTab);

            % Create KinematicAnimationPanel
            app.KinematicAnimationPanel = uipanel(app.GridLayout19);
            app.KinematicAnimationPanel.Title = 'Kinematic Animation';
            app.KinematicAnimationPanel.Layout.Row = 1;
            app.KinematicAnimationPanel.Layout.Column = 2;

            % Create GridLayout6
            app.GridLayout6 = uigridlayout(app.KinematicAnimationPanel);

            % Create AnimateKineButton
            app.AnimateKineButton = uibutton(app.GridLayout6, 'push');
            app.AnimateKineButton.ButtonPushedFcn = createCallbackFcn(app, @AnimateKineButtonPushed, true);
            app.AnimateKineButton.Layout.Row = 2;
            app.AnimateKineButton.Layout.Column = 1;
            app.AnimateKineButton.Text = 'Animate Kine';

            % Create GivePosesButton
            app.GivePosesButton = uibutton(app.GridLayout6, 'push');
            app.GivePosesButton.ButtonPushedFcn = createCallbackFcn(app, @GivePosesButtonPushed, true);
            app.GivePosesButton.Layout.Row = 1;
            app.GivePosesButton.Layout.Column = 2;
            app.GivePosesButton.Text = {'Give Poses'; ''};

            % Create FindSingularitiesButton
            app.FindSingularitiesButton = uibutton(app.GridLayout6, 'push');
            app.FindSingularitiesButton.ButtonPushedFcn = createCallbackFcn(app, @FindSingularitiesButtonPushed, true);
            app.FindSingularitiesButton.Layout.Row = 2;
            app.FindSingularitiesButton.Layout.Column = 2;
            app.FindSingularitiesButton.Text = 'Find Singularities';

            % Create ShowPosesButton
            app.ShowPosesButton = uibutton(app.GridLayout6, 'push');
            app.ShowPosesButton.ButtonPushedFcn = createCallbackFcn(app, @ShowPosesButtonPushed, true);
            app.ShowPosesButton.Layout.Row = 1;
            app.ShowPosesButton.Layout.Column = 1;
            app.ShowPosesButton.Text = 'Show Poses';

            % Create JointConfigurationsPanel
            app.JointConfigurationsPanel = uipanel(app.GridLayout19);
            app.JointConfigurationsPanel.Title = 'Joint Configurations';
            app.JointConfigurationsPanel.Layout.Row = 2;
            app.JointConfigurationsPanel.Layout.Column = 1;
            app.JointConfigurationsPanel.Scrollable = 'on';

            % Create GridLayout20
            app.GridLayout20 = uigridlayout(app.JointConfigurationsPanel);
            app.GridLayout20.ColumnWidth = {'1x'};
            app.GridLayout20.RowHeight = {'1x'};

            % Create kineConfigTable
            app.kineConfigTable = uitable(app.GridLayout20);
            app.kineConfigTable.ColumnName = {'Col 1'};
            app.kineConfigTable.RowName = {};
            app.kineConfigTable.ColumnEditable = true;
            app.kineConfigTable.Layout.Row = 1;
            app.kineConfigTable.Layout.Column = 1;

            % Create GridLayout33
            app.GridLayout33 = uigridlayout(app.GridLayout19);
            app.GridLayout33.ColumnWidth = {'1x'};
            app.GridLayout33.Padding = [0 0 0 0];
            app.GridLayout33.Layout.Row = 1;
            app.GridLayout33.Layout.Column = 1;

            % Create ForwardKinematicsOptionsPanel
            app.ForwardKinematicsOptionsPanel = uipanel(app.GridLayout33);
            app.ForwardKinematicsOptionsPanel.Title = 'Forward Kinematics Options';
            app.ForwardKinematicsOptionsPanel.Layout.Row = 1;
            app.ForwardKinematicsOptionsPanel.Layout.Column = 1;
            app.ForwardKinematicsOptionsPanel.Scrollable = 'on';

            % Create GridLayout5
            app.GridLayout5 = uigridlayout(app.ForwardKinematicsOptionsPanel);
            app.GridLayout5.RowHeight = {'1x'};

            % Create ConfigurationNumberEditFieldLabel
            app.ConfigurationNumberEditFieldLabel = uilabel(app.GridLayout5);
            app.ConfigurationNumberEditFieldLabel.HorizontalAlignment = 'right';
            app.ConfigurationNumberEditFieldLabel.Layout.Row = 1;
            app.ConfigurationNumberEditFieldLabel.Layout.Column = 1;
            app.ConfigurationNumberEditFieldLabel.Text = {'Configuration Number'; ''};

            % Create ConfigurationNumberEditField
            app.ConfigurationNumberEditField = uieditfield(app.GridLayout5, 'numeric');
            app.ConfigurationNumberEditField.Limits = [0 20];
            app.ConfigurationNumberEditField.RoundFractionalValues = 'on';
            app.ConfigurationNumberEditField.ValueChangedFcn = createCallbackFcn(app, @ConfigurationNumberEditFieldValueChanged, true);
            app.ConfigurationNumberEditField.Layout.Row = 1;
            app.ConfigurationNumberEditField.Layout.Column = 2;
            app.ConfigurationNumberEditField.Value = 1;

            % Create DifferentialKinematicsOptionsPanel
            app.DifferentialKinematicsOptionsPanel = uipanel(app.GridLayout33);
            app.DifferentialKinematicsOptionsPanel.Title = 'Differential Kinematics Options';
            app.DifferentialKinematicsOptionsPanel.Layout.Row = 2;
            app.DifferentialKinematicsOptionsPanel.Layout.Column = 1;

            % Create GridLayout34
            app.GridLayout34 = uigridlayout(app.DifferentialKinematicsOptionsPanel);

            % Create SampleFrequencyEditFieldLabel
            app.SampleFrequencyEditFieldLabel = uilabel(app.GridLayout34);
            app.SampleFrequencyEditFieldLabel.HorizontalAlignment = 'right';
            app.SampleFrequencyEditFieldLabel.Layout.Row = 2;
            app.SampleFrequencyEditFieldLabel.Layout.Column = 1;
            app.SampleFrequencyEditFieldLabel.Text = {'Sample Frequency'; ''};

            % Create SampleFrequencyEditField
            app.SampleFrequencyEditField = uieditfield(app.GridLayout34, 'numeric');
            app.SampleFrequencyEditField.Limits = [0.001 Inf];
            app.SampleFrequencyEditField.Layout.Row = 2;
            app.SampleFrequencyEditField.Layout.Column = 2;
            app.SampleFrequencyEditField.Value = 1000;

            % Create FinalTimeEditField_2Label
            app.FinalTimeEditField_2Label = uilabel(app.GridLayout34);
            app.FinalTimeEditField_2Label.HorizontalAlignment = 'right';
            app.FinalTimeEditField_2Label.Layout.Row = 1;
            app.FinalTimeEditField_2Label.Layout.Column = 1;
            app.FinalTimeEditField_2Label.Text = {'Final Time'; ''};

            % Create diffFinalTimeEditField
            app.diffFinalTimeEditField = uieditfield(app.GridLayout34, 'numeric');
            app.diffFinalTimeEditField.Limits = [0 Inf];
            app.diffFinalTimeEditField.Layout.Row = 1;
            app.diffFinalTimeEditField.Layout.Column = 2;
            app.diffFinalTimeEditField.Value = 1;

            % Create SingularitiesPanel
            app.SingularitiesPanel = uipanel(app.GridLayout19);
            app.SingularitiesPanel.Title = 'Singularities';
            app.SingularitiesPanel.Layout.Row = 2;
            app.SingularitiesPanel.Layout.Column = 2;

            % Create GridLayout35
            app.GridLayout35 = uigridlayout(app.SingularitiesPanel);
            app.GridLayout35.ColumnWidth = {'1x'};
            app.GridLayout35.RowHeight = {'1x'};

            % Create singularTable
            app.singularTable = uitable(app.GridLayout35);
            app.singularTable.ColumnName = {'Single Link Index'; 'Singular Angle'; 'Coupled Link Index 1'; 'Singluar Angle'; 'Coupled Link Index 2'; 'Singular Angle'};
            app.singularTable.RowName = {};
            app.singularTable.Layout.Row = 1;
            app.singularTable.Layout.Column = 1;

            % Create InverseKinematicsTab
            app.InverseKinematicsTab = uitab(app.TabGroup2);
            app.InverseKinematicsTab.Title = 'Inverse Kinematics';

            % Create GridLayout21
            app.GridLayout21 = uigridlayout(app.InverseKinematicsTab);

            % Create InverseKinematicAnimationPanel
            app.InverseKinematicAnimationPanel = uipanel(app.GridLayout21);
            app.InverseKinematicAnimationPanel.Title = 'Inverse Kinematic Animation';
            app.InverseKinematicAnimationPanel.Layout.Row = 1;
            app.InverseKinematicAnimationPanel.Layout.Column = 2;

            % Create GridLayout13
            app.GridLayout13 = uigridlayout(app.InverseKinematicAnimationPanel);

            % Create AnimateInvKineButton
            app.AnimateInvKineButton = uibutton(app.GridLayout13, 'push');
            app.AnimateInvKineButton.ButtonPushedFcn = createCallbackFcn(app, @AnimateInvKineButtonPushed, true);
            app.AnimateInvKineButton.Layout.Row = 2;
            app.AnimateInvKineButton.Layout.Column = 1;
            app.AnimateInvKineButton.Text = {'Animate InvKine'; ''};

            % Create AnimateInvKineJacoButton
            app.AnimateInvKineJacoButton = uibutton(app.GridLayout13, 'push');
            app.AnimateInvKineJacoButton.ButtonPushedFcn = createCallbackFcn(app, @AnimateInvKineJacoButtonPushed, true);
            app.AnimateInvKineJacoButton.Layout.Row = 2;
            app.AnimateInvKineJacoButton.Layout.Column = 2;
            app.AnimateInvKineJacoButton.Text = {'Animate InvKine'; ' with Jacobian'; ''; ''};

            % Create GiveConfigurationsButton
            app.GiveConfigurationsButton = uibutton(app.GridLayout13, 'push');
            app.GiveConfigurationsButton.ButtonPushedFcn = createCallbackFcn(app, @GiveConfigurationsButtonPushed, true);
            app.GiveConfigurationsButton.Layout.Row = 1;
            app.GiveConfigurationsButton.Layout.Column = 2;
            app.GiveConfigurationsButton.Text = {'Give Configurations'; ''};

            % Create ShowConfigurationsButton
            app.ShowConfigurationsButton = uibutton(app.GridLayout13, 'push');
            app.ShowConfigurationsButton.ButtonPushedFcn = createCallbackFcn(app, @ShowConfigurationsButtonPushed, true);
            app.ShowConfigurationsButton.Layout.Row = 1;
            app.ShowConfigurationsButton.Layout.Column = 1;
            app.ShowConfigurationsButton.Text = 'Show Configurations';

            % Create EndEffectorPosePanel
            app.EndEffectorPosePanel = uipanel(app.GridLayout21);
            app.EndEffectorPosePanel.Title = 'End Effector Pose';
            app.EndEffectorPosePanel.Layout.Row = 2;
            app.EndEffectorPosePanel.Layout.Column = 1;

            % Create GridLayout22
            app.GridLayout22 = uigridlayout(app.EndEffectorPosePanel);
            app.GridLayout22.ColumnWidth = {'1x'};
            app.GridLayout22.RowHeight = {'1x'};

            % Create invKineConfigTable
            app.invKineConfigTable = uitable(app.GridLayout22);
            app.invKineConfigTable.ColumnName = {'Col 1'};
            app.invKineConfigTable.RowName = {};
            app.invKineConfigTable.ColumnEditable = true;
            app.invKineConfigTable.Layout.Row = 1;
            app.invKineConfigTable.Layout.Column = 1;

            % Create GridLayout36
            app.GridLayout36 = uigridlayout(app.GridLayout21);
            app.GridLayout36.ColumnWidth = {'1x'};
            app.GridLayout36.Padding = [0 0 0 0];
            app.GridLayout36.Layout.Row = 1;
            app.GridLayout36.Layout.Column = 1;

            % Create InverseKinematicsOptionsPanel
            app.InverseKinematicsOptionsPanel = uipanel(app.GridLayout36);
            app.InverseKinematicsOptionsPanel.Title = 'Inverse Kinematics Options';
            app.InverseKinematicsOptionsPanel.Layout.Row = 1;
            app.InverseKinematicsOptionsPanel.Layout.Column = 1;

            % Create GridLayout7
            app.GridLayout7 = uigridlayout(app.InverseKinematicsOptionsPanel);
            app.GridLayout7.RowHeight = {'1x'};

            % Create PoseNumberLabel
            app.PoseNumberLabel = uilabel(app.GridLayout7);
            app.PoseNumberLabel.HorizontalAlignment = 'right';
            app.PoseNumberLabel.Layout.Row = 1;
            app.PoseNumberLabel.Layout.Column = 1;
            app.PoseNumberLabel.Text = 'Pose Number';

            % Create PoseNumberEditField
            app.PoseNumberEditField = uieditfield(app.GridLayout7, 'numeric');
            app.PoseNumberEditField.Limits = [0 20];
            app.PoseNumberEditField.RoundFractionalValues = 'on';
            app.PoseNumberEditField.ValueChangedFcn = createCallbackFcn(app, @PoseNumberEditFieldValueChanged, true);
            app.PoseNumberEditField.Layout.Row = 1;
            app.PoseNumberEditField.Layout.Column = 2;
            app.PoseNumberEditField.Value = 1;

            % Create DifferentialKinematicsOptionsPanel_2
            app.DifferentialKinematicsOptionsPanel_2 = uipanel(app.GridLayout36);
            app.DifferentialKinematicsOptionsPanel_2.Title = 'Differential Kinematics Options';
            app.DifferentialKinematicsOptionsPanel_2.Layout.Row = 2;
            app.DifferentialKinematicsOptionsPanel_2.Layout.Column = 1;

            % Create GridLayout34_2
            app.GridLayout34_2 = uigridlayout(app.DifferentialKinematicsOptionsPanel_2);

            % Create SampleFrequencyEditField_3Label
            app.SampleFrequencyEditField_3Label = uilabel(app.GridLayout34_2);
            app.SampleFrequencyEditField_3Label.HorizontalAlignment = 'right';
            app.SampleFrequencyEditField_3Label.Layout.Row = 2;
            app.SampleFrequencyEditField_3Label.Layout.Column = 1;
            app.SampleFrequencyEditField_3Label.Text = {'Sample Frequency'; ''};

            % Create SampleFrequencyEditField_3
            app.SampleFrequencyEditField_3 = uieditfield(app.GridLayout34_2, 'numeric');
            app.SampleFrequencyEditField_3.Limits = [0.001 Inf];
            app.SampleFrequencyEditField_3.Layout.Row = 2;
            app.SampleFrequencyEditField_3.Layout.Column = 2;
            app.SampleFrequencyEditField_3.Value = 1000;

            % Create FinalTimeEditField_2Label_3
            app.FinalTimeEditField_2Label_3 = uilabel(app.GridLayout34_2);
            app.FinalTimeEditField_2Label_3.HorizontalAlignment = 'right';
            app.FinalTimeEditField_2Label_3.Layout.Row = 1;
            app.FinalTimeEditField_2Label_3.Layout.Column = 1;
            app.FinalTimeEditField_2Label_3.Text = {'Final Time'; ''};

            % Create diffFinalTimeEditField_3
            app.diffFinalTimeEditField_3 = uieditfield(app.GridLayout34_2, 'numeric');
            app.diffFinalTimeEditField_3.Limits = [0 Inf];
            app.diffFinalTimeEditField_3.Layout.Row = 1;
            app.diffFinalTimeEditField_3.Layout.Column = 2;
            app.diffFinalTimeEditField_3.Value = 1;

            % Create DynamicsTab
            app.DynamicsTab = uitab(app.tabGroupMain);
            app.DynamicsTab.Title = 'Dynamics';

            % Create InputsPanel
            app.InputsPanel = uipanel(app.DynamicsTab);
            app.InputsPanel.Title = 'Inputs';
            app.InputsPanel.Scrollable = 'on';
            app.InputsPanel.Position = [0 -2 211 394];

            % Create FinalTimeLabel
            app.FinalTimeLabel = uilabel(app.InputsPanel);
            app.FinalTimeLabel.HorizontalAlignment = 'right';
            app.FinalTimeLabel.Position = [16 342 64 22];
            app.FinalTimeLabel.Text = 'Final Time:';

            % Create FinalTimeEditField
            app.FinalTimeEditField = uieditfield(app.InputsPanel, 'numeric');
            app.FinalTimeEditField.Position = [95 342 100 22];
            app.FinalTimeEditField.Value = 1;

            % Create InitialPositionPanel
            app.InitialPositionPanel = uipanel(app.InputsPanel);
            app.InitialPositionPanel.Title = 'Initial Position';
            app.InitialPositionPanel.Scrollable = 'on';
            app.InitialPositionPanel.Position = [13 128 186 112];

            % Create GridLayout9
            app.GridLayout9 = uigridlayout(app.InitialPositionPanel);
            app.GridLayout9.ColumnWidth = {'1x'};
            app.GridLayout9.RowHeight = {'1x'};

            % Create q0Table
            app.q0Table = uitable(app.GridLayout9);
            app.q0Table.ColumnName = {''};
            app.q0Table.RowName = {'q0'};
            app.q0Table.ColumnEditable = true;
            app.q0Table.Layout.Row = 1;
            app.q0Table.Layout.Column = 1;

            % Create TorqueFunctionOptionsPanel
            app.TorqueFunctionOptionsPanel = uipanel(app.InputsPanel);
            app.TorqueFunctionOptionsPanel.Title = 'Torque Function Options';
            app.TorqueFunctionOptionsPanel.Scrollable = 'on';
            app.TorqueFunctionOptionsPanel.Position = [13 239 186 94];

            % Create TypeDropDownLabel
            app.TypeDropDownLabel = uilabel(app.TorqueFunctionOptionsPanel);
            app.TypeDropDownLabel.HorizontalAlignment = 'right';
            app.TypeDropDownLabel.Position = [18 48 35 22];
            app.TypeDropDownLabel.Text = 'Type:';

            % Create TypeDropDown
            app.TypeDropDown = uidropdown(app.TorqueFunctionOptionsPanel);
            app.TypeDropDown.Items = {'Constant', 'PD', 'Custom'};
            app.TypeDropDown.ValueChangedFcn = createCallbackFcn(app, @TypeDropDownValueChanged, true);
            app.TypeDropDown.Position = [68 48 100 22];
            app.TypeDropDown.Value = 'Constant';

            % Create NameEditFieldLabel
            app.NameEditFieldLabel = uilabel(app.TorqueFunctionOptionsPanel);
            app.NameEditFieldLabel.HorizontalAlignment = 'right';
            app.NameEditFieldLabel.Position = [12 18 41 22];
            app.NameEditFieldLabel.Text = 'Name:';

            % Create NameEditField
            app.NameEditField = uieditfield(app.TorqueFunctionOptionsPanel, 'text');
            app.NameEditField.Position = [68 18 100 22];
            app.NameEditField.Value = '@exampleTorqfun';

            % Create InitialVelocityPanel
            app.InitialVelocityPanel = uipanel(app.InputsPanel);
            app.InitialVelocityPanel.Title = 'Initial Velocity';
            app.InitialVelocityPanel.Scrollable = 'on';
            app.InitialVelocityPanel.Position = [13 4 186 125];

            % Create GridLayout10
            app.GridLayout10 = uigridlayout(app.InitialVelocityPanel);
            app.GridLayout10.ColumnWidth = {'1x'};
            app.GridLayout10.RowHeight = {'1x'};

            % Create qd0Table
            app.qd0Table = uitable(app.GridLayout10);
            app.qd0Table.ColumnName = {''};
            app.qd0Table.RowName = {'qd0'};
            app.qd0Table.ColumnEditable = true;
            app.qd0Table.Layout.Row = 1;
            app.qd0Table.Layout.Column = 1;

            % Create TabGroup
            app.TabGroup = uitabgroup(app.DynamicsTab);
            app.TabGroup.SelectionChangedFcn = createCallbackFcn(app, @TabGroupSelectionChanged, true);
            app.TabGroup.Position = [210 106 430 286];

            % Create JointPositionsandVelocitiesTab
            app.JointPositionsandVelocitiesTab = uitab(app.TabGroup);
            app.JointPositionsandVelocitiesTab.Title = 'Joint Positions and Velocities';

            % Create DynamicsPlotsPanel
            app.DynamicsPlotsPanel = uipanel(app.JointPositionsandVelocitiesTab);
            app.DynamicsPlotsPanel.Title = 'Dynamics Plots';
            app.DynamicsPlotsPanel.Scrollable = 'on';
            app.DynamicsPlotsPanel.Position = [1 0 429 261];

            % Create GridLayout8
            app.GridLayout8 = uigridlayout(app.DynamicsPlotsPanel);
            app.GridLayout8.ColumnWidth = {'1x'};
            app.GridLayout8.RowHeight = {'200x', '200x'};

            % Create axesJointPositions
            app.axesJointPositions = uiaxes(app.GridLayout8);
            title(app.axesJointPositions, 'Joint Positions')
            xlabel(app.axesJointPositions, 'Time')
            ylabel(app.axesJointPositions, 'Position')
            app.axesJointPositions.PlotBoxAspectRatio = [7.2 1 1];
            app.axesJointPositions.Layout.Row = 1;
            app.axesJointPositions.Layout.Column = 1;

            % Create axesJointVelocities
            app.axesJointVelocities = uiaxes(app.GridLayout8);
            title(app.axesJointVelocities, 'Joint Velocities')
            xlabel(app.axesJointVelocities, 'Time')
            ylabel(app.axesJointVelocities, 'Velocity')
            app.axesJointVelocities.PlotBoxAspectRatio = [7.2 1 1];
            app.axesJointVelocities.Layout.Row = 2;
            app.axesJointVelocities.Layout.Column = 1;

            % Create EquationsofMotionTab
            app.EquationsofMotionTab = uitab(app.TabGroup);
            app.EquationsofMotionTab.Title = 'Equations of Motion';

            % Create GridLayout31
            app.GridLayout31 = uigridlayout(app.EquationsofMotionTab);
            app.GridLayout31.ColumnWidth = {'1x'};
            app.GridLayout31.RowHeight = {'1x', '1x', '1x', '1x'};

            % Create Image
            app.Image = uiimage(app.GridLayout31);
            app.Image.Layout.Row = 1;
            app.Image.Layout.Column = 1;
            app.Image.ImageSource = 'MotionEQ.png';

            % Create eqMotionInputTable
            app.eqMotionInputTable = uitable(app.GridLayout31);
            app.eqMotionInputTable.ColumnName = {'Column 1'; 'Column 2'; 'Column 3'; 'Column 4'};
            app.eqMotionInputTable.RowName = {};
            app.eqMotionInputTable.ColumnEditable = true;
            app.eqMotionInputTable.Layout.Row = 2;
            app.eqMotionInputTable.Layout.Column = 1;

            % Create GridLayout32
            app.GridLayout32 = uigridlayout(app.GridLayout31);
            app.GridLayout32.RowHeight = {'1x'};
            app.GridLayout32.Padding = [0 0 0 0];
            app.GridLayout32.Layout.Row = 3;
            app.GridLayout32.Layout.Column = 1;

            % Create EOMSelectDropDown
            app.EOMSelectDropDown = uidropdown(app.GridLayout32);
            app.EOMSelectDropDown.Items = {'B', 'C', 'F', 'G', 'J', ''};
            app.EOMSelectDropDown.Layout.Row = 1;
            app.EOMSelectDropDown.Layout.Column = 2;
            app.EOMSelectDropDown.Value = 'B';

            % Create DisplaySelectedMatrixButton
            app.DisplaySelectedMatrixButton = uibutton(app.GridLayout32, 'push');
            app.DisplaySelectedMatrixButton.ButtonPushedFcn = createCallbackFcn(app, @DisplaySelectedMatrixButtonPushed, true);
            app.DisplaySelectedMatrixButton.Layout.Row = 1;
            app.DisplaySelectedMatrixButton.Layout.Column = 1;
            app.DisplaySelectedMatrixButton.Text = 'Display Selected Matrix';

            % Create eqMotionOutputTable
            app.eqMotionOutputTable = uitable(app.GridLayout31);
            app.eqMotionOutputTable.ColumnName = {'Column 1'; 'Column 2'; 'Column 3'; 'Column 4'};
            app.eqMotionOutputTable.RowName = {};
            app.eqMotionOutputTable.Layout.Row = 4;
            app.eqMotionOutputTable.Layout.Column = 1;

            % Create ControlPanel
            app.ControlPanel = uipanel(app.DynamicsTab);
            app.ControlPanel.Title = 'Control';
            app.ControlPanel.Position = [481 -1 158 107];

            % Create CalculateDynamicsButton
            app.CalculateDynamicsButton = uibutton(app.ControlPanel, 'push');
            app.CalculateDynamicsButton.ButtonPushedFcn = createCallbackFcn(app, @CalculateDynamicsButtonPushed, true);
            app.CalculateDynamicsButton.Position = [13 53 122 22];
            app.CalculateDynamicsButton.Text = 'Calculate Dynamics';

            % Create AnimateDynamicsButton
            app.AnimateDynamicsButton = uibutton(app.ControlPanel, 'push');
            app.AnimateDynamicsButton.ButtonPushedFcn = createCallbackFcn(app, @AnimateDynamicsButtonPushed, true);
            app.AnimateDynamicsButton.Position = [13 16 122 22];
            app.AnimateDynamicsButton.Text = 'Animate Dynamics';

            % Create InputsPanel_2
            app.InputsPanel_2 = uipanel(app.DynamicsTab);
            app.InputsPanel_2.Title = 'Inputs';
            app.InputsPanel_2.Position = [210 -4 272 110];

            % Create GridLayout11
            app.GridLayout11 = uigridlayout(app.InputsPanel_2);
            app.GridLayout11.ColumnWidth = {'1x'};
            app.GridLayout11.RowHeight = {'1x'};

            % Create torqFunTable
            app.torqFunTable = uitable(app.GridLayout11);
            app.torqFunTable.ColumnName = {'Column 1'; 'Column 2'; 'Column 3'; 'Column 4'};
            app.torqFunTable.RowName = {};
            app.torqFunTable.ColumnEditable = true;
            app.torqFunTable.Layout.Row = 1;
            app.torqFunTable.Layout.Column = 1;

            % Create ControlTab
            app.ControlTab = uitab(app.tabGroupMain);
            app.ControlTab.Title = 'Control';

            % Create Panel
            app.Panel = uipanel(app.UIFigure);
            app.Panel.Position = [2 1 638 61];

            % Create GridLayout12
            app.GridLayout12 = uigridlayout(app.Panel);
            app.GridLayout12.ColumnWidth = {'1x'};
            app.GridLayout12.RowHeight = {'1x'};

            % Create ConsoleTextArea
            app.ConsoleTextArea = uitextarea(app.GridLayout12);
            app.ConsoleTextArea.Editable = 'off';
            app.ConsoleTextArea.FontName = 'Lucida Sans';
            app.ConsoleTextArea.Layout.Row = 1;
            app.ConsoleTextArea.Layout.Column = 1;
            app.ConsoleTextArea.Value = {'Hello and welcome!'};

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GUI_Beta_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupCallback)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end