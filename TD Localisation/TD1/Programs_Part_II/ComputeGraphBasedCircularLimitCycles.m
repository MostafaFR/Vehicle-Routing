%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%% Program inspired from ref [RAS 17] (which should be cited if any   %
%% use for research and/or academic works).                           %  
%% [RAS 17], L. Adouane, Reactive versus cognitive vehicle navigation %
%% based on optimal local and global PELC*. Robotics and Autonomous   %
%% Systems (RAS), v 88, pp. 51�70, February 2017.                     %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Last modified on 21/03/2022                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function MainTree = ComputeGraphBasedCircularLimitCycles()
global Obstacle %List of obstacles available in the environment
global CriteriaValueOfNotAccessiblePoint
global RobotTricycle
global Bornes_Mu NbreInterationsMax MainTree %%For data backup

%%Recovery of the current vehicle posture and assignment for the start of optimization
RobotPosture = GetPosture(RobotTricycle);
Xreel0 = RobotPosture(1); Yreel0=RobotPosture(2); Theta0 = RobotPosture(3); Gamma0=RobotPosture(4);   %Initial conditions of the robot

%%Constant to define the type of plot and visualization
bAffichage = 1; bAnimation = 0; WhichColor = 'Blue'; LineWidth = 0.5; LineStyle = '-';
PlotFeatures = struct('bAffichage', bAffichage, 'bAnimation', bAnimation, 'Color', WhichColor, 'Width', LineWidth, 'LineStyle', LineStyle);

%%Parameters of the Dichotomy research
% Bornes_Mu = [2 12];       %%Concerns the search interval of Mu
Bornes_Mu = [4 12]; 
NbreInterationsMax = 5;   %%Number of additional loops for dichotomy search (in addition to the standard ones [Min, (Min + Max) / 2, Max])

%%Initialisation for the first node of the Tree
ID_Obstcale = 0;            %Convention for the initial vehicle posture
bClockwise = 0;             %Is the direction adopted to reach the node (by convention, the original node is attributed bClockwise = 0)
ValeurMu = 0; %Is the value of Alpha to reach the node (by convention, the original node is attributed Alpha = 0)
PELCCost = 0;               %Is the value of Cost to reach the node (by convention, the original node is attributed Cost = 0)
PELCCostDetails = [0 0 0];  %[DistanceToFinalSetPoint TrajectoryCurvilinearDistance SommeCuravture];
DataGain = [0 0 0];         %[K_Criteria K_Curvateur K_MainTarget];
NodeData0 = [Xreel0 , Yreel0, Theta0, Gamma0, ID_Obstcale, bClockwise, ValeurMu, PELCCost PELCCostDetails(1) PELCCostDetails(2) PELCCostDetails(3) DataGain(1) DataGain(2) DataGain(3)]; %%[x, y, theta, gamma, ID_Obstcale, bClockwise, ValeurMu, Cost]
MainTree = tree(NodeData0); %%Initialization of the Tree with a root (the intial node)
%%%%%%%%%%%%%%%%%%%%%%%%
ID_Target = size(Obstacle, 2);   %%Getting the target identifier
%%%
SetIndexesNodesCurrentLevel = 1; %%Which corresponds initially to the root node
while (~isempty(SetIndexesNodesCurrentLevel)) %%While it exists at least one node in this vector which means that we can extend more the Tree to find a solution to reach the Final target
    SetIndexesNodesCurrentLevel_Temp = []; %%Initialization to save the IDs of the nodes to extend the Tree for the next pass
    for i_NodesCurrentLevel = SetIndexesNodesCurrentLevel
        TempVector = MainTree.get(i_NodesCurrentLevel);  % Get the data of this node
        RobotInitialConfiguration = TempVector(1:4);     % Get the initial configuration of the robot for the corresponding iteration
        %%Iterate for one LC
        IndexesObstrctingObstacles = [];
        for j=1:2 %% Loop for iterating from of nodes with different bClockwise and ValeurMu (this former is iterated by the function "GetPELC_OptimalFeatures")
            if (j==1)
                bClockwise =1;
            else
                bClockwise =0;
            end
            
            [Alpha_Optimal PELC_Optimal_Cost Vector_ID_CollidedObstcales RobotTrajectory DataOptimalCostDetails DataGain] = ...
                GetOptimal_LC_Features(Bornes_Mu, NbreInterationsMax, RobotInitialConfiguration, ...
                ID_Target, bClockwise, PlotFeatures);
            
            %%Adding a node only if an optimal path to the target can be found from the initial robot configuration
            if (PELC_Optimal_Cost ~= CriteriaValueOfNotAccessiblePoint) %%If the optimal solution is really achievable
                NodeData = [RobotTrajectory(end, 1:4), ID_Target, bClockwise, Alpha_Optimal, PELC_Optimal_Cost DataOptimalCostDetails(1) DataOptimalCostDetails(2) DataOptimalCostDetails(3) DataGain(1) DataGain(2) DataGain(3)]; %%[x, y, theta, gamma, ID_Obstcale, bClockwise, ValeurMu, Cost]
                MainTree = MainTree.addnode(i_NodesCurrentLevel, NodeData); %% Attach to the initial node (the immediate Parent)
            end
            %%Moreover, if there are more configuration of detected collisions, it would be necessary to find the right limit cycles towards these annoying obstacles  
            %%from the initial node configuration
            if (~isnan(Vector_ID_CollidedObstcales))
                NumberObstructingObstacles = size(Vector_ID_CollidedObstcales, 2);
                for iObstructingObstacles=1:NumberObstructingObstacles
                    ID_CollidedObstcale = Vector_ID_CollidedObstcales(iObstructingObstacles); %%Retrieve the ID of the annoying obstacle
                    TempVector = MainTree.get(i_NodesCurrentLevel); %[RobotTrajectory(end, :), ID_Target, bClockwise, Alpha_Optimal, PELC_Optimal_Cost];
                    DepartureNode_ID_Obstcale = TempVector(5);
                    if ( (~(any( IndexesObstrctingObstacles == ID_CollidedObstcale ))) && (DepartureNode_ID_Obstcale ~= ID_CollidedObstcale ) )   %%If the obstacle is not already addressed
                        for k=1:2
                            if (k==1)
                                bClockwise =1;
                            else
                                bClockwise =0;
                            end
                            IndexesObstrctingObstacles = [IndexesObstrctingObstacles ID_CollidedObstcale]; %%Add this index to the already addressed collision
                            [Alpha_Optimal PELC_Optimal_Cost Vector_ID_CollidedObstcales_bis RobotTrajectory DataOptimalCostDetails DataGain] = ...
                                GetOptimal_LC_Features(Bornes_Mu, NbreInterationsMax, RobotInitialConfiguration, ...
                                ID_CollidedObstcale, bClockwise, PlotFeatures);
                            
                            if (PELC_Optimal_Cost ~= CriteriaValueOfNotAccessiblePoint) %%If the optimal solution is really achievable
                                NodeData = [RobotTrajectory(end, 1:4), ID_CollidedObstcale, bClockwise, Alpha_Optimal, PELC_Optimal_Cost DataOptimalCostDetails(1) DataOptimalCostDetails(2) DataOptimalCostDetails(3) DataGain(1) DataGain(2) DataGain(3)]; %%[x, y, theta, gamma, ID_Obstcale, bClockwise, ValeurMu, Cost]
                                [MainTree ID_Node] = MainTree.addnode(i_NodesCurrentLevel, NodeData); %% Attach to the initial node (the immediate Parent)
                                SetIndexesNodesCurrentLevel_Temp = [SetIndexesNodesCurrentLevel_Temp ID_Node];
                            end
                        end %%End Number of config. from each initial config. w.r.t specific obstacle
                    end %%End If for address the case of the colliding obstacle
                end %%End number of obstructing obstacles
            end  %%End else there is a collision
        end %%End Number of config. from each initial config. w.r.t specific obstacle
    end %%End for the number of current nodes of the current level (iteration on i_NodesCurrentLevel)
    SetIndexesNodesCurrentLevel = SetIndexesNodesCurrentLevel_Temp; %%Final attribution of the different Nodes to extend for the next iteration
end %%End While (SetIndexesNodesCurrentLevel ~= []) %%

save MainTree;           %%Case where we need to Save the obtained Tree
%load('MainTree');       %%Case where we need to Load the already obtained Tree
disp(MainTree.tostring); %%Showing the obtained Tree