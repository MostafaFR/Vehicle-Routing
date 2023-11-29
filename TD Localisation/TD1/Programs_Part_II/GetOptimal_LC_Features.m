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

function [Alpha_Optimal PELC_Optimal_Cost Vector_ID_CollidedObstcales ...
    RobotTrajectory DataOptimalCostDetails DataGain] = ...
    GetOptimal_LC_Features(Bornes_Mu, NbreInterationsMax, RobotInitialConfiguration, ...
    ID_Obstacle, bClockwise, PlotFeatures)
%Obtaining using Dichotomy of the Optimal LC between an initial vehicle configuration and an obstacle/target in the environment (and this
%according to one direction of the rotation (clockwise or counter-clockwise)).

global CriteriaValueOfNotAccessiblePoint

Vector_ID_CollidedObstcales  = []; %%Initialization of the Vector which count the collided obstacles during the computation of the LC*

%%Dichotomy algo initialization
%%Parameters of the dichotomy algorithm (Mu variation interval and number of iterations of the algorithm)
BorneInf_Mu = Bornes_Mu(1);
BorneSup_Mu = Bornes_Mu(2);

%%Boundary (Borne) Low
Mu_i = BorneInf_Mu;
[CriteriaValue_BorneInf ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Mu_i, PlotFeatures);

%%Add or not of the possible ID_CollidedObstcale
if ( (~(any( Vector_ID_CollidedObstcales == ID_CollidedObstcale ))) && (ID_CollidedObstcale ~=  0) ) %%If the obstacle is not already addressed, or it exists effectively
    Vector_ID_CollidedObstcales = [Vector_ID_CollidedObstcales ID_CollidedObstcale];
end
sauv_Mu_i(1, :) = [Mu_i, CriteriaValue_BorneInf];

%%Boundary (Borne) High
Mu_i = BorneSup_Mu;
[CriteriaValue_BorneSup ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Mu_i, PlotFeatures);
if ( (~(any( Vector_ID_CollidedObstcales == ID_CollidedObstcale ))) && (ID_CollidedObstcale ~=  0) ) %%If the obstacle is not already addressed, or it exists effectively
    Vector_ID_CollidedObstcales = [Vector_ID_CollidedObstcales ID_CollidedObstcale];
end
sauv_Mu_i(2, :) = [Mu_i, CriteriaValue_BorneSup];

%%Beginning of Dichotomy algo if we do a lot of iterations
CriteriaValue_BorneInter_OLD = 1000; %Initialization with a big value
for i = 1:NbreInterationsMax
    %%Boundary intermediate
    BorneInter_Alpha = (BorneSup_Mu + BorneInf_Mu)/2;
    Mu_i = BorneInter_Alpha;
    [CriteriaValue_BorneInter ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Mu_i, PlotFeatures);
    if ( (~(any( Vector_ID_CollidedObstcales == ID_CollidedObstcale ))) && (ID_CollidedObstcale ~=  0) ) %%If the obstacle is not already addressed, or it exists effectivelly
        Vector_ID_CollidedObstcales = [Vector_ID_CollidedObstcales ID_CollidedObstcale];
    end
    sauv_Mu_i(2+i, :) = [Mu_i, CriteriaValue_BorneInter];
    
    if (CriteriaValue_BorneInf <= CriteriaValue_BorneSup)
        BorneSup_Mu = BorneInter_Alpha;
        CriteriaValue_BorneSup = CriteriaValue_BorneInter;
    else
        BorneInf_Mu = BorneInter_Alpha;
        CriteriaValue_BorneInf = CriteriaValue_BorneInter;
    end
    %%Not mainly important here, but in case of...
    if (BorneSup_Mu < BorneInf_Mu) %%Switch the order of boundary if the relation of order is not satisfied
        Borne_temp = BorneInf_Mu;
        BorneInf_Mu = BorneSup_Mu;
        BorneSup_Mu = Borne_temp;
    end
    %%Test to go outside this loop when the criteria do not change a lot
    if ((CriteriaValue_BorneInter_OLD - CriteriaValue_BorneInter) < 0.05)
        break;
    else
        CriteriaValue_BorneInter_OLD = CriteriaValue_BorneInter;
    end
end

%%The last iteration while dividing the interval by 2 (Not so important here!)
BorneInter_Alpha = (BorneSup_Mu + BorneInf_Mu)/2;
Mu_i = BorneInter_Alpha;
[CriteriaValue_BorneInter ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Mu_i, PlotFeatures);
if ( (~(any( Vector_ID_CollidedObstcales == ID_CollidedObstcale ))) && (ID_CollidedObstcale ~=  0) ) %%If the obstacle is not already addressed, or it exists effectivelly
    Vector_ID_CollidedObstcales = [Vector_ID_CollidedObstcales ID_CollidedObstcale];
end
sauv_Mu_i(3+i, :) = [Mu_i, CriteriaValue_BorneInter];

%%Obtaining of the output of the function
Alpha_Optimal = sauv_Mu_i(1, 1); %%Choose to start, the first element, and check with others
CriteriaValue_Optimal = sauv_Mu_i(1, 2);
for j=2:(3+i)
    CriteriaValue_Optimal_temp = sauv_Mu_i(j, 2); %%Get back the value of the corresponding value of the criteria
    if (CriteriaValue_Optimal_temp < CriteriaValue_Optimal)
        Alpha_Optimal = sauv_Mu_i(j, 1);
        CriteriaValue_Optimal = sauv_Mu_i(j, 2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Display or not of the result of the optimal local intermediate solution (compared to a single obstacle) if it exists !!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bAffichage = 0; bAnimation = 0; WhichColor = 'Blue'; LineWidth = 0.5; LineStyle = '-';
%%PlotFeatures = struct('bAffichage', bAffichage, 'bAnimation', bAnimation, 'Color', WhichColor, 'Width', LineWidth);
PlotFeatures = struct('bAffichage', bAffichage, 'bAnimation', bAnimation, 'Color', WhichColor, 'Width', LineWidth, 'LineStyle', LineStyle);

[PELC_Optimal_Cost ID_CollidedObstcale RobotTrajectory DataOptimalCostDetails DataGain] = Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Alpha_Optimal, PlotFeatures);

if (CriteriaValue_Optimal ~= CriteriaValueOfNotAccessiblePoint)
    plot(RobotTrajectory(:,1), RobotTrajectory(:,2), 'LineWidth', 1.5 , 'lineStyle', '--' ,'Color', 'blue');
    Module = 0.30;
    quiver(RobotTrajectory(end,1), RobotTrajectory(end,2), Module*cos(RobotTrajectory(end,3)),...
        Module*sin(RobotTrajectory(end,3)),'MaxHeadSize', 0.9, 'LineStyle','-','LineWidth', 2,'Color',[1 0 0], 'Marker', 'o', ...
        'MarkerSize', 4); %%, 'MarkerFaceColor', [1 0 0]
    Alpha_Optimal_bis= Alpha_Optimal;
    Alpha_Optimal_ = sprintf('%.2f', Alpha_Optimal_bis); %%1 - Pour garder la logique énoncée pour Mu
    PELC_Optimal_Cost_ = sprintf('%.2f', PELC_Optimal_Cost);
    PT = ['\mu =' Alpha_Optimal_ ', J=' PELC_Optimal_Cost_];% ', J=' num2str(PELC_Optimal_Cost_)];
    
    text( RobotTrajectory(end, 1)+Module*cos(RobotTrajectory(end,3)), RobotTrajectory(end, 2)+Module*sin(RobotTrajectory(end,3)), ...
        PT, 'FontSize', 8, 'LineWidth', 6, 'Color', [0 0 0]);
end