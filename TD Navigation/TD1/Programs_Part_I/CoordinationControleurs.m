%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part I : Planning and control of mobile robots                     %
%% Th�or�me de stabilit� de Lyapunov et m�thode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
function Sortie = CoordinationControleurs(Data)
 
global IndicateurCA IndicateurCA_OLD
global Xd Yd
global Obstacle
global TempsCourant
global PasEchantillonnage %Le pas d'�chantillonnage constant
global inc2 Sauvinc2
 
global S_DistanceRobotCible S_AngleRobotCible S_DistanceRobotObstacle1  S_DistanceVerticleRobotObstacle1  S_IndiceObstaclePlusProche S_DifferenceDistanceObstacles S_SensEvitement
global SemaphoreTempsImpactCible
global SensEvitementObstacle_Old     %Pour sauvegarder le sens de l'�vitement d'obstacle pour l'it�ration d'apr�s (-1 -> counter-clockwise, 1 -> clockwise and 0 don't avoiding before or else)
global IndiceObstaclePlusProche_Old  %Pour sauvegarder de l'indice de l'obstacle sur lequel s'applique l'�vitement
global G_ActivationAlgorithmeOrbital %Variable globale pour l'activation (==1) ou non de l'algorithme orbital
 
%%Etat actuel du robot mobile
Xreel = Data(1);
Yreel = Data(2);
Theta = Data(3);
%%Initialisations
DistanceRobotObstaclePlusProche = 10000; %Une valeur trop grande qui devrait �tre mise � jour s'il y a effectivement un obstacle trop proche
IndiceObstaclePlusProche = 0 ;           %Si cette variable reste � Z�ro, c'est qu'il n'y a pas de collision probable avec un obstacle
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Trouver les obstacles qui peuvent rentrer en collision avec le robot s'il part tout droit vers l'objectif
%%"Le but �tant d'isoler les obstacles susceptibles de g�ner l'avanc�e du robot vers la cible"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initialistions 
IndiceObstacle =1; 
ListeObstaclesCollisionPotentiel = [];
 
for i=1:size(Obstacle, 2)
    PositionObstaclei = GetPosition(Obstacle(i));
    Rvi = GetRv(Obstacle(i));
    %%Distance Robot-Obstacle
    DistanceRO = DistanceEuclidienne(PositionObstaclei, [Xreel Yreel]);
    if  (DistanceRO <= Rvi) 
        ListeObstaclesCollisionPotentiel(IndiceObstacle)=i;
        IndiceObstacle = IndiceObstacle + 1;
    end
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Trouver l'obstacle le plus proche et calcul de la distance entre l'obstacle et le robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isempty(ListeObstaclesCollisionPotentiel) 
    IndiceObstaclePlusProche = ListeObstaclesCollisionPotentiel(1);
    DistanceRobotObstacle_Min = DistanceEuclidienne(GetPosition(Obstacle(IndiceObstaclePlusProche)), [Xreel Yreel]);
    DistanceObstacleCible_ObstacleAyantLeMinROCourant = DistanceEuclidienne(GetPosition(Obstacle(IndiceObstaclePlusProche)), [Xd Yd]);
    for i=2:size(ListeObstaclesCollisionPotentiel, 2) 
        IndiceObstaclei = ListeObstaclesCollisionPotentiel(i);
        DistanceRobotObstaclei = DistanceEuclidienne(GetPosition(Obstacle(IndiceObstaclei)), [Xreel Yreel]);
        %%
        if (DistanceRobotObstaclei < DistanceRobotObstacle_Min)  
            IndiceObstaclePlusProche_old =  IndiceObstaclePlusProche;
            DistanceRobotObstacle_Min_old = DistanceRobotObstacle_Min;
            IndiceObstaclePlusProche = IndiceObstaclei;
            DistanceRobotObstacle_Min = DistanceRobotObstaclei;
        end
    end
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Force activation of the controller "Attraction to the Target"  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IndicateurCA=1;
% DonneesCommandeAttraction = VariablesCommandeAttraction(Data);
% Commande  = CommandeAttraction(DonneesCommandeAttraction);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%Selection Process 
%%%%%%%%%%%%%%%%%%%%%%%%%
SensEvitementObstacle = 0; %i.e., que le contr�leur d'�vitement doit trouver par lui-m�me le sens qu'il doit prendre
DonneesCommandeEvitement = [0; 0; 0];  
if isempty(ListeObstaclesCollisionPotentiel)
    IndicateurCA = 0; %Activer le contr�leur d'attraction
    DonneesCommandeAttraction = VariablesCommandeAttraction(Data);
else
    IndicateurCA = 1; %Activer le controleur d'�vitement d'obstacles
    DonneesCommandeEvitement = VariablesCommandeEvitement(Data, IndiceObstaclePlusProche, DistanceRobotObstaclePlusProche, SensEvitementObstacle);    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Calcul de la commande � appliquer aux actionneurs%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (IndicateurCA==0) %Cas de l'attraction vers l'objectif
    Commande  = CommandeAttraction(DonneesCommandeAttraction);
else %Activation de l'�vitement d'obstacles
    Commande =  CommandeEvitement(DonneesCommandeEvitement);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%Sauvegarde de la commande actuelle et de l'indicateur de l'ancien contr�leur actif pour une utilisation ult�rieure
IndicateurCA_OLD = IndicateurCA; 
CommandeReelle_OLD = Commande;  
%Sortie de la fonction 
Sortie = [Commande IndicateurCA];
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Sauvegarde des variables caract�risant l'�volution du robot par rapport �
%l'objectif et � l'obstacle � �viter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
inc2 = inc2+1;
S_DistanceRobotCible(:,inc2) = DistanceEuclidienne([Xreel Yreel], [Xd Yd]);
if (S_DistanceRobotCible(:,inc2) <= 0.2) & (SemaphoreTempsImpactCible == 1)
    inc2;            %Affichage ou non de l'incr�ment
    Sauvinc2 = inc2; %Variable pour sauvegarder le moment d'impact du robot avec le cercle entourant la cible 
    SemaphoreTempsImpactCible = 0;
end
 
%%Angle robot cible
ex = Xreel-Xd;
ey = Yreel-Yd;
S_AngleRobotCible(:,inc2) = Theta - atan2(ey,ex);