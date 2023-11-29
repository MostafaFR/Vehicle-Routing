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

function [LCCost ID_CollidedObstcale RobotTrajectory DataCostDetails DataGain] = ...
    Get_LC_Features(RobotInitialConfiguration, ID_Obstacle, bClockwise, Mu_i, PlotFeatures)
%%Function to move the robot according to limit cycles set-points
global Obstacle
global PasEchantillonnageCommande
global SemaphoreSauvPositionInitiale
global CriteriaValueOfNotAccessiblePoint
global K_Criteria K_Curvateur K_MainTarget

%%Fonctions d'affichage
bAffichage = PlotFeatures.bAffichage; bAnimation = PlotFeatures.bAnimation ; WhichColor = PlotFeatures.Color;
LineWidth = PlotFeatures.Width; LineStyle = PlotFeatures.LineStyle;

%Getting the features of the Obstacle around that the LC Traj. is obtained
X0 = GetX(Obstacle(ID_Obstacle));
Y0 = GetY(Obstacle(ID_Obstacle));
A = GetA(Obstacle(ID_Obstacle));  %%Important : Dans ces programmes fournis, il faudra toujours mettre A = B (pour avoir des obstacles toujours circulaires)
Marge = GetMarge(Obstacle(ID_Obstacle));

AngleEntreAxesRepereObstacle = GetAngleEntreAxesRepere(Obstacle(ID_Obstacle)); %%% Récupération des caractéristiques du repère lié à l'obstacle à éviter
CentreRepereEvitement = GetCentreRepere(Obstacle(ID_Obstacle)); %%C'est le centre du repère lié à l'obstacle
PointsTangentes = GetPointsTangentes(Obstacle(ID_Obstacle));    %%Get Points from the Final Target

%Initialisation de la structure robot [x, y, theta, Gamma] dédiée à cette fonction
TempRobotTricycle = CRobotTricycle(RobotInitialConfiguration);

SemaphoreSauvPositionInitiale = 0; %%VERY IMPORTANT TO INITIELIZE AT EACH CALL OF THE FUNCTIONS

bReachRepereObstacle = 0; %% Boolean variable to tell if the obtained traj. reaches "==1" (or not ==0) the reference frame of the obstacle to avoid
bCollision = 0;           %% Which must be 0 after the for loop, if at least one obstacle collide with the robot
ID_CollidedObstcale = 0;  %% Save the ID of the collided obstacle "By default, ID == 0 if no collided obstacle"
RobotTrajectory = [];
NombreIterationsMax = 6000; %%C'est le nombre d'itérations max que peut avoir le cycle-limite à obtenir
inc = 0;
LinearVelocity = 0;
SignePostionRobot_X_OLD = 1;

for i = 1:NombreIterationsMax
    inc = inc+1;
    RobotPosture = GetPosture(TempRobotTricycle);
    RobotTrajectory(inc, :) = [RobotPosture LinearVelocity];    %%Save the traj.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Obtention de la consigne [ModuleVitesse AngleVitesse] à injecter au niveau de la commande
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%Calcul de l'orientation "ThetaC" à faire suivre par le robot pour suivre le cycle limite
    
    %Etat actuel du Robot Mobile
    Xreel = RobotPosture(1);
    Yreel = RobotPosture(2);
    
    %%Détermination de la position relative du robot par rapport à l'obstacle
    Xrelatif = Xreel - X0;
    Yrelatif = Yreel - Y0;
    
    RayonCycleLimite = A + Marge;
    
    %%Pour définir le sens de rotation "clockwise" ou "counter-clockwise"
    if (bClockwise==1)
        Y_Prime = 1;
    else
        Y_Prime = -1;
    end
    
    ID_Target = size(Obstacle, 2);   %%Getting of the target identifier
    if (ID_Obstacle ~= ID_Target)
        %%Calcul du cycle-limite
        X_dot = sign(Y_Prime) * Yrelatif + (Mu_i * Xrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2)));
        Y_dot = - sign(Y_Prime) * Xrelatif + (Mu_i * Yrelatif * ((RayonCycleLimite^2) - (Xrelatif^2) - (Yrelatif^2)));
    else
        %%Pour orienter directement la trajectoire vers la cible
        X_dot =  X0 - Xreel;
        Y_dot =  Y0 - Yreel;
    end
    
    %%Obtention du module de vitesse et angle
    ModuleVitesse = 0.2; %% Vitesse d'évolution constante pour la planification
    AngleVitesse  = atan2(Y_dot, X_dot);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%Application du contrôleur angulaire simple proportionnel
    K = 1; %gain par rapport à la commande angulaire
    ValeurSoustraction = SoustractionAnglesAtan2(AngleVitesse, RobotPosture(3));%% ThetaC, Theta AngleVitesse-Theta
    DeltaTheta = K*(ValeurSoustraction)*PasEchantillonnageCommande; %%Proportional controller !
    
    RobotPosture(3) = AngleVitesse;
    RobotPosture(1) = RobotPosture(1) + PasEchantillonnageCommande*ModuleVitesse*cos(RobotPosture(3));
    RobotPosture(2) = RobotPosture(2) + PasEchantillonnageCommande*ModuleVitesse*sin(RobotPosture(3));
    %%%
    TempRobotTricycle = SetPosture(TempRobotTricycle, RobotPosture);
    
    LinearVelocity = PasEchantillonnageCommande*ModuleVitesse; %%Cette ligne est là juste pour la sauvegarde des consignes des vitesses, utilisable lors de la restitution de la courbe de vitesse
    
    RobotPosture = GetPosture(TempRobotTricycle);
    Position =  PositionRobotDansRepereObstacle(RobotPosture(1), RobotPosture(2), AngleEntreAxesRepereObstacle, CentreRepereEvitement);
    %%%%
    SignePostionRobot_X = sign(Position(1));
    if (((SignePostionRobot_X_OLD <=0) && (SignePostionRobot_X > 0)) && (inc >1)) %%Pour dire qu'on switch uniquement quand on passe du signe moins au plus et non une autre combinaison
        RayonCible = 0.1; %%C'est le rayon à partir duquel on considère que la cible est atteinte
        D = sqrt(Position(1)^2+Position(2)^2); %%On calcul ainsi cette distance car Position est donnée par rapport au centre du repère de l'obstacle
        if ( ( (bClockwise == 1)&&(sign(Position(2)) >= 0) ) || ( (bClockwise == 0)&&(sign(Position(2)) <= 0) ) || (D <= RayonCible) )
            bReachRepereObstacle = 1;
        end
        break;
    end
    SignePostionRobot_X_OLD = sign(Position(1));
    
    %%Boolean information if collision or not with at least one obstacle is in the environment, if so, put "0" to the criteria which means, no way to go
    %%using this traj. %bCollision = 0; %%Wich must be 0 after the for loop, if at least one obstacle collide the trajectory (supposed rectilinear here)
    for ID_Obstacle_temp=1:1:(size(Obstacle, 2)-1) % "-1" is to remove the Target from the list
        X0_ = GetX(Obstacle(ID_Obstacle_temp));
        Y0_ = GetY(Obstacle(ID_Obstacle_temp));
        Center_ = [X0_ Y0_];
        A_ = GetA(Obstacle(ID_Obstacle_temp));
        B_ = GetB(Obstacle(ID_Obstacle_temp));
        Phi_ = GetPhi(Obstacle(ID_Obstacle_temp));
        Marge_ = GetMarge(Obstacle(ID_Obstacle_temp));
        Marge_ =  Marge_ - 0.01; %%The minus is the tolerance
        if ((IsRobotInEnveloppe(RobotPosture(1), RobotPosture(2), Center_, A_, B_, Phi_, Marge_)))
            bCollision  = 1;
            ID_CollidedObstcale = ID_Obstacle_temp;
            break; %%Sortir de la boucle de test de la collision avec tous les obstacles
        end
    end %%Fin boucle pour le test si le robot rentre en collision avec un des obstacles dans l'environnement
    if (bCollision == 1) %% Ce break c'est pour sortir cette fois-ci de la boucle globale qui permet d'obtenir le cycle-limite
        break; %%Break from the FOR loop to obtain the trajectory
    end
    %%%%%%%%%%%%%%%%%%%%%
end
%%% Save of the last robot configuration
RobotTrajectory(inc+1, :) = [GetPosture(TempRobotTricycle) LinearVelocity];

%%%%%%%%%%%%%%%%%%%%%%%
%%Criteria Evaluation%%
%%%%%%%%%%%%%%%%%%%%%%%
%%1) Criteria based on the length of the obtained traj.
TrajectoryCurvilinearDistance = LongueurAPartirEnsemblePoints(RobotTrajectory(:,1:2));

%%2) Criteria based on the final configuration of the robot (position and orientation)
Robot_FinalX = RobotTrajectory(end,1);
Robot_FinalY = RobotTrajectory(end,2);
%%Position of the set-point given by the point
if bClockwise
    Robot_FinalSetPointX = PointsTangentes(1, 1);
    Robot_FinalSetPointY =  PointsTangentes(2, 1);
else
    Robot_FinalSetPointX =  PointsTangentes(1, 2);
    Robot_FinalSetPointY =  PointsTangentes(2, 2);
end
DistanceToFinalSetPoint = sqrt((Robot_FinalX - Robot_FinalSetPointX)^2 + (Robot_FinalY - Robot_FinalSetPointY)^2);

%%%3) Criteria linked to the curvature
[Curvature SommeCuravture CurvatureMax] = GetTrajectoryCurvature(RobotTrajectory(:,1:2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Modification of the values of the gains of the Global criteria J if the targeted LC is according to the Main Target, in this case, it is
%%important to have more weight for the distance to the Target.
ID_Target = size(Obstacle, 2);   %%Getting of the target identifier
if (ID_Obstacle ~= ID_Target)
    K_MainTarget = 0;
else
    K_MainTarget = 10;
end
K_Criteria = 0.01;
K_Curvateur = 0.0005;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Attribution of the value of the criteria
DistanceMaxToFinalIntermidiateSetPoint = 2; %If we do not want to penalize the end-point which is too far from the attributed point
if ((bCollision ~= 1) && (bReachRepereObstacle == 1) && (DistanceToFinalSetPoint < DistanceMaxToFinalIntermidiateSetPoint))  %%If not any collision along the trajectory or the final point of the traj. is too far from the intermediate target
    %%% LCCost = K_Criteria*(DistanceToFinalSetPoint) + (1-K_Criteria)*TrajectoryCurvilinearDistance ...
    %%% + K_Curvateur*SommeCuravture + K_MainTarget*(DistanceToFinalSetPoint);
    
    LCCost = TrajectoryCurvilinearDistance; % SommeCuravture + DistanceToFinalSetPoint;
    DataCostDetails = [DistanceToFinalSetPoint TrajectoryCurvilinearDistance SommeCuravture];
    DataGain  = [K_Criteria K_Curvateur  K_MainTarget];
else
    LCCost = CriteriaValueOfNotAccessiblePoint; %A big value, so that this way will never be chosen by the optimal path search!
    DataCostDetails = [CriteriaValueOfNotAccessiblePoint CriteriaValueOfNotAccessiblePoint SommeCuravture];
    DataGain = [K_Criteria K_Curvateur K_MainTarget];
end
%%%%%%%%%%%%%%END OF CRITERIA EVALUATION

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Affichage ou pas du résultat de la solution intermédiaire locale (par rapport à un seul obstacle) optimale si elle existe !!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (bAffichage && (LCCost ~= CriteriaValueOfNotAccessiblePoint) )
    hold on
    plot(RobotTrajectory(:,1), RobotTrajectory(:,2),'LineWidth', LineWidth , 'lineStyle', LineStyle ,'Color', WhichColor);
    Module = 0.30;
    quiver(RobotTrajectory(end,1), RobotTrajectory(end,2), Module*cos(RobotTrajectory(end,3)),...
        Module*sin(RobotTrajectory(end,3)),'MaxHeadSize', 0.9, 'LineStyle','-', 'LineWidth', 2,'Color',[1 0 0], 'Marker', 'o', ...
        'MarkerSize', 4);
    Mu_i_ = sprintf('%.2f', Mu_i);
    LCCost_Cost_ = sprintf('%.2f', LCCost);
    PT = ['\mu =' Mu_i_ ', J=' LCCost_Cost_];
end