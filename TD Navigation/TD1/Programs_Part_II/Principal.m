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
%% Etudiant :
%% Date :
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all, clc, clear all,
%clear classes
global Xreel0 Yreel0 Theta0     %% Initial conditions of the robot
global Xreel Yreel Theta        %% Current robot location
global Xd Yd Rd                 %% Coordinates of the desired point to reach et le rayon de la main target
global Obstacle                 %% Declaration of the list of obstacles
global PasEchantillonnage PasEchantillonnageCommande
 
global MargeDefaut              %% Default margin around obstacles
global ConfigurationObstacles
global Exi                      %% Setting a small value for the whole program
 
global CriteriaValueOfNotAccessiblePoint %%If there is a collision for example while taking this trajectory
global RobotTricycle      %%Declaration of the Tricycle robot
global Xmin_axis Xmax_axis Ymin_axis Ymax_axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialization of the access path to the libraries used for the operation of the Software
addpath('./linecurvature_version1b'); %%For the calculation of the curvatures of a set of input points / lines
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialization of constants and Lists %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TricyclePrameters(); %%Function where is all the robot parameters, it is a global function given in the main folder
CriteriaValueOfNotAccessiblePoint = inf; %%A big value so that the Edge will not be chosen!
 
Exi = 0.00001; 
MargeDefaut = Rrobot + 0.14;
 
%%%Initial conditions of the robot
Xreel0 = 0.5; Yreel0 = 1.5; Theta0 = deg2rad(45); Gamma0 = 0;
Xreel = Xreel0; Yreel = Yreel0;  Theta = Theta0; Gamma = Gamma0;
%%%Coordinates in the environment of the point to be reached and of the radius of the main target
Xd =5; Yd =3.6;   %%Position of the final target to be reached
Rd = Rrobot;    %%Target radius
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Part to cause the graph to be displayed or not
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bRepereObstacleCible = 1;
bRepereObstacleRobot = 0;
bArreterSimulationSiCollision = 0;
bChampPerception = 1; %%Variables to display or not some simulation elements related to obstacles
bMarge = 1;
bID = 1; %%Obstacle identification display
bAffichageCaracteristiquesTrajectoireRobot = 1; %%Display of the smoothness criteria of the trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xmin_axis = 0; Xmax_axis = 8; Ymin_axis = -0.5; Ymax_axis = 5;
axis([Xmin_axis Xmax_axis Ymin_axis Ymax_axis])
 
%%Declaration of obstacles in the environment (here, it will always be necessary to put A = B -> The obstacles are surrounded by a circular envelope)
ConfigurationObstacles = [2, 2, ... %%X et Y (1)
    0.2, 0.2, ...   %%A et B
    0, MargeDefaut; %%Phi, Marge
    4.1, 2.9, ... %%X et Y (1)%ADDED
    0.2, 0.2, ...   %%A et B
    0, MargeDefaut; %%Phi, Marge
    3, 2.7, ... %%X et Y (1) %ADDED
    0.2, 0.2, ...   %%A et B
    0, MargeDefaut; %%Phi, Marge
    3.7, 1.75, ...   %%X et Y (2)
    0.3, 0.3, ...   %%A et B
    0, MargeDefaut; %%Phi, Marge
    5, 2.4, ...   %%X et Y (3)
    0.1, 0.1, ...   %%A et B
    0, MargeDefaut; %%Phi, Marge
    Xd, Yd, ...     %%X et Y (4) %%The last is always the target (assimilated to an obstacle with a negligible radius (so there will still be an attraction))
    0.001, 0.001, ... %%A et B
    0 , 0];           %%Phi, Marge
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Instantiation of the used obstacles and robots of the simulation%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Obstacle = CObstacleP();%Initialization of the structure
for i=1:size(ConfigurationObstacles, 1)
    Obstacle(i) = CObstacleP(ConfigurationObstacles(i,1:2), ConfigurationObstacles(i, 3),...
        ConfigurationObstacles(i, 4), ConfigurationObstacles(i,5), ConfigurationObstacles(i,6), ...
        [0.0, 0.0], Xreel0, Yreel0); %%CObstacle(Position, A, B, Phi, Marge, Vitesse, Xrobot, Yrobot)
end
%%Instantiation of the used tricycle. In the control part of this LAB,
%%the robot could be considered as unicycle to simplify the
%%synthesis of the control law (based Lyapunov function) 
RobotTricycle = CRobotTricycle([Xreel0 Yreel0 Theta0 Gamma0]);     %%Initialization of the robot structure [x, y, theta, Gamma]
TempRobotTricycle = CRobotTricycle([Xreel0 Yreel0 Theta0 Gamma0]); %%Initialization of the robot structure [x, y, theta, Gamma]
 
%%%%%%%%%%%%%%%%%%%%%
%%%Initial display%%%
%%%%%%%%%%%%%%%%%%%%%
Simulation_type = 1;
%%Display of the different elements of the simulation
close all,
Figure1=figure(Simulation_type); %To create the figure on which the results are displayed
axes1 = axes('Parent', Figure1, 'FontSize',14);%% Create axes
hold on; grid on; axis equal;
%bMarge = 0; bChampPerception = 0; bRepereObstacleCible = 0;  bRepereObstacleRobot = 0; bID =0;
for i=1:size(Obstacle, 2) %%Affichage de l'ensemble des obstacles
    Affichage(Obstacle(i), bChampPerception, bMarge, bRepereObstacleCible, ...
        bRepereObstacleRobot, bID, Simulation_type); %%Affichage(Obstacle, bChampPerception, bMarge, bRepereObstacle, bRepereObstacleRobot, bID, iFigure)
end
DessinTricycle(Xreel0, Yreel0, Theta0, Gamma0, 1, ' '); %%The '' is to add text if you want
 
plot(Xreel0,Yreel0,'k+','LineWidth',1); %%Linked to the initial robot config
plot(Xd,Yd,'k+','LineWidth',3.5);       %%Linked to the main target config
 
xlabel('X [m]', 'fontsize', 14);
ylabel('Y [m]', 'fontsize', 14);
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%Construction of the graph based on circular limit cycles (To be adapted according to your needs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Parameters for the period of simulations
TempsSimulationMax = 100;         
TempsCourant = 0;
PasEchantillonnage = 0.01;        %Constant sampling time for simulation in Simulink 
PasEchantillonnageCommande = 0.1; %Sampling time dedicated to robot control and the evolution of its kinematic model
%%%
MainTree = ComputeGraphBasedCircularLimitCycles();  %%Simplified program that generates the graph of the different possibilities (Important: To be adapted according to your actual need)
TrajectorySetPoints = TreeProcess(MainTree);        %%Simplified program which allows you to find the shortest path from the overall graph obtained (Important: To be adapted according to your actual need)
