%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% SY28 :: Département Génie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%% Program inspired from ref [RAS 17] (which should be cited if any   %
%% use for research and/or academic works).                           %  
%% [RAS 17], L. Adouane, Reactive versus cognitive vehicle navigation %
%% based on optimal local and global PELC*. Robotics and Autonomous   %
%% Systems (RAS), v 88, pp. 51–70, February 2017.                     %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Last modified on 21/03/2022                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TrajectorySetPoints = TreeProcess(MainTree)

%%Function to process and plot the obtained Tree
global Obstacle
global Xmin_axis Xmax_axis Ymin_axis Ymax_axis
global RobotTricycle

%%%Recovery of the current robot posture and assignment to start the optimization
RobotPosture = GetPosture(RobotTricycle);
Xreel0 = RobotPosture(1); Yreel0=RobotPosture(2); Theta0 = RobotPosture(3); Gamma0=RobotPosture(4);   %%Initial conditions of the robot

ID_Target = size(Obstacle, 2);   %%Getting of the target identifier

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%View the tree of successive ID_Obstacles LCcosts after making a copy/synchronization of the overall Tree
TempTree = tree(MainTree, 'clear');      %% Copy the tree structure only
iterator = MainTree.depthfirstiterator;  %% Return a line vector of indices that traverse the tree in a depth-first manner, starting from the root node
for i = iterator
    GlobalData = MainTree.get(i);
    TempTree = TempTree.set(i, ['Node=' num2str(i) ',ID=' num2str(GlobalData(5)) ',Cost=' num2str(GlobalData(8))]); %%',tCost='  num2str(tCost) Set the value, where %%[Xreel0 , Yreel0, Theta0, Gamma0, ID_Obstcale, bClockwise, Mu_i, PELCCost]
end
disp(TempTree.tostring)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Getting the sequence of the optimal way-points
VectorIndexesCostResult = [];
IndexLeavesMinCost = [];
LeavesIndexes = findleaves(MainTree); %%Permit us to obtain the vector of terminal nodes (Leaves)

%%Selection only the Leaves which terminate at the reference frame linked to the target
tempLeavesIndexes = [];
for i=LeavesIndexes
    GlobalData = MainTree.get(i);
    if (GlobalData(5) == ID_Target) %%To maintain only the Leaves reaching the target
        tempLeavesIndexes = [tempLeavesIndexes i];
    end
end
LeavesIndexes = tempLeavesIndexes;

%% Compute the sum of all elementary costs and the index of the optimal solution
Costpath2root_OLD = 100000; %%Initialized with a big value in order to not take this cost if any other obtained cost for an obtained path
Costpath2root_DistanceToFinalSetPoint_OLD = 10000;
Costpath2root_TrajectoryCurvilinearDistance_OLD = 10000;
Costpath2root_SommeCuravture_OLD = 10000;
OptimalSolution = 'Global' %% 'Global' 'DistanceToFinalSetPoint' 'TrajectoryCurvilinearDistance' 'SommeCuravture'
for i=LeavesIndexes
    path2root = findpath2root( MainTree, i);
    Costpath2root = 0; %%Initialization
    Costpath2root_DistanceToFinalSetPoint = 0;
    Costpath2root_TrajectoryCurvilinearDistance = 0;
    Costpath2root_SommeCuravture = 0;
    
    for j=path2root
        GlobalData = MainTree.get(j);
        Costpath2root = Costpath2root + GlobalData(8);
        Costpath2root_DistanceToFinalSetPoint = Costpath2root_DistanceToFinalSetPoint + ((GlobalData(12)+GlobalData(14)) * GlobalData(9));
        Costpath2root_TrajectoryCurvilinearDistance = Costpath2root_TrajectoryCurvilinearDistance + ((1-GlobalData(12)) * GlobalData(10));
        Costpath2root_SommeCuravture = Costpath2root_SommeCuravture + (GlobalData(13) * GlobalData(11));
    end
    VectorIndexesCostResult = [VectorIndexesCostResult;  i Costpath2root Costpath2root_DistanceToFinalSetPoint ...
        Costpath2root_TrajectoryCurvilinearDistance Costpath2root_SommeCuravture]; % Save the ID of the Leaf and its cost to be reached from the root node
    
    %%Choice of which optimal solution to take (according to the criteria of the overall cost function)
    switch OptimalSolution
        case {'Global'}
            if (Costpath2root < Costpath2root_OLD) %%If to save the index of the minimal node cost
                IndexLeavesMinCost = i;
                Costpath2root_OLD = Costpath2root;
            end
        case {'DistanceToFinalSetPoint'}
            if (Costpath2root_DistanceToFinalSetPoint < Costpath2root_DistanceToFinalSetPoint_OLD) %%If to save the index of the minimal node cost
                IndexLeavesMinCost = i;
                Costpath2root_DistanceToFinalSetPoint_OLD = Costpath2root_DistanceToFinalSetPoint;
            end
        case {'TrajectoryCurvilinearDistance'}
            if (Costpath2root_TrajectoryCurvilinearDistance < Costpath2root_TrajectoryCurvilinearDistance_OLD) %%If to save the index of the minimal node cost
                IndexLeavesMinCost = i;
                Costpath2root_TrajectoryCurvilinearDistance_OLD = Costpath2root_TrajectoryCurvilinearDistance;
            end
        case {'SommeCuravture'}
            if (Costpath2root_SommeCuravture < Costpath2root_SommeCuravture_OLD) %%If to save the index of the minimal node cost
                IndexLeavesMinCost = i;
                Costpath2root_SommeCuravture_OLD = Costpath2root_SommeCuravture;
            end
        otherwise
            disp('Unknown optimal optimization')
    end
end

%%Display of the obtained results
disp('========================');
disp('The used gains [K_Criteria K_Curvateur  K_MainTarget]')
GlobalData = MainTree.get(LeavesIndexes(1));
GlobalData(12:14)
disp('Simulation solution');
disp('Leaves = ID_Node || Ovreall PELCCost || Cost_DistanceToFinalSetPoint  || Cost_TrajectoryCurvilinearDistance || Cost_SommeCuravture');
VectorIndexesCostResult %Vector of the nodes indexes and corresponding cost from each Leaf backward to the root node
disp('The optimal Leaf');
IndexLeavesMinCost %Index of the Leaf with minimal cost
%%%%

%%Obtaining of the sequence of nodes of the minimal solution
Optimalpath2root = findpath2root( MainTree, IndexLeavesMinCost);
Optimalpath2root = fliplr(Optimalpath2root); %%Reverse the vector "Optimalpath2root" to start from the initial configuration of the vehicle to the corresponding Leaf

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%Getting of the optimal sequence of waypoints "Waypoint = [Obstacle ID || bClockwise || Mu]";
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Waypoints = [];
for i=Optimalpath2root(2:end)
    %%[Xreel0 , Yreel0, Theta0, Gamma0, ID_Obstcale, bClockwise, Mu_i, PELCCost]
    GlobalData = MainTree.get(i);
    Obtscale_ID = GlobalData(5);
    bClockwise = GlobalData(6);
    Mu_i = GlobalData(7);
    Waypoints = [Waypoints; Obtscale_ID, bClockwise, Mu_i];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Restitution of the Trajectory that should be followed by the robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialization to do in order to avoid to deal with the same obstacle a multitude of time (when we have for example two Waypoints in the same robot)
bAffichage = 1; bAnimation = 0; WhichColor = 'green'; LineWidth = 2; LineStyle = '-.';
PlotFeatures = struct('bAffichage', bAffichage, 'bAnimation', bAnimation, 'Color', [0 1 0], 'Width', LineWidth, 'LineStyle', LineStyle);

TrajectorySetPoints = [];
IndiceObstaclePlusProche_old = 0;
RobotInitialConfiguration = [Xreel0 Yreel0 Theta0 Gamma0]; %%Initialization of the robot configuration_0

for i=1:size(Waypoints, 1) %%Start by 2 to do not consider the initial configuration of the robot!
    IndiceObstaclePlusProche = Waypoints(i, 1);
    if (IndiceObstaclePlusProche ~= IndiceObstaclePlusProche_old) %%To filter the points which are in the same obstacle (use only one point as a reference for the obstacle)
        IndiceObstaclePlusProche_old = IndiceObstaclePlusProche; %%Update of the index of the current obstacle avoidance
        bClockwise = Waypoints(i, 2);
        Mu= Waypoints(i, 3);  %%It is the Mu_i
        [PELCCost ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = ...
            Get_LC_Features(RobotInitialConfiguration, IndiceObstaclePlusProche, bClockwise, Mu, PlotFeatures);
        %%Update the new initial configuration from the last obtained traj.
        RobotInitialConfiguration = RobotTrajectory(end,1:4); %[sauv_x_robot(end) sauv_y_robot(end) sauv_theta_robot(end) ];
        %%Ajout de la nouvelle trajectoire
        TrajectorySetPoints = [TrajectorySetPoints; RobotTrajectory];
    end
end

MotifTrajPELC = plot(100, 100, '-', 'LineWidth', 1.5, 'Color',[0 0 1]);
MotifTrajgPELC_Optimale = plot(100, 100, '-', 'LineWidth', 2, 'Color',[0 0.5 0]);

legend1 = legend([MotifTrajPELC MotifTrajgPELC_Optimale], {'Elementary planned LC', 'Global optimal gLC*'}); %'FontWeight','bold' , 'FontSize',10
set(legend1,'FontSize',14);
axis([Xmin_axis Xmax_axis Ymin_axis Ymax_axis])