%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% SY28 :: Département Génie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Affichage(Obstacle, bChampPerception, bMarge, bRepereObstacleCible, bRepereObstacleRobot, bID, iFigure)
global Xd Yd
global Xreel Yreel
global Xmin_axis Xmax_axis Ymin_axis Ymax_axis
persistent semaphore figObstacle1 figObstacle2 figObstacle3 figObstacle4 figObstacle5 figObstacle6 TextObstacle1

Obstacle_ID = Obstacle.ID;

%% Plot
if (~isempty(semaphore) && (Obstacle.Vitesse(1)~= 0) && (Obstacle.Vitesse(2)~= 0) )
    delete(figObstacle1(Obstacle_ID))
    delete(figObstacle2(Obstacle_ID))
    delete(figObstacle3(Obstacle_ID))
    delete(figObstacle4(Obstacle_ID))
    delete(figObstacle5(Obstacle_ID))
    delete(figObstacle6(Obstacle_ID))
    delete(TextObstacle1(Obstacle_ID))
end
semaphore = 1;

%Détermination de Xobstacle = f(Yobstacle) (contour de l'obstacle réel)
gamma = [0:0.01:2*pi]';

A = Obstacle.A;
B = Obstacle.B;
PositionObstacle = Obstacle.Position;

Xobstacle = A * cos(gamma);
Yobstacle = B * sin(gamma);

PointsTransformes=Trans([PositionObstacle(1), PositionObstacle(2), 0])*Rotz(Obstacle.Phi)*...
    [Xobstacle'; Yobstacle'; zeros(1, size(Xobstacle', 2)); ones(1, size(Xobstacle',2))];

Xobstacle = PointsTransformes(1, :);
Yobstacle = PointsTransformes(2, :);

figObstacle1(Obstacle_ID)=fill(Xobstacle,Yobstacle, 'k'); % Remplissage de la couleur de la base

%Tracé de l'obstacle
figure(iFigure)
hold on

figObstacle2(Obstacle_ID) = plot(Obstacle.Position(1), Obstacle.Position(2), 'r+','LineWidth',1);
hold on
figObstacle3(Obstacle_ID) = plot(Xobstacle,Yobstacle,'k','LineWidth',2);

axis equal
grid on

if bRepereObstacleCible
    
    Pt_intersection = Obstacle.CentreRepere;
    Angle_Cible_AxeYObstacle = Obstacle.AngleEntreAxesRepereObstacle;
    
    X_D_O = Xd - Pt_intersection(1);
    Y_D_O = Yd - Pt_intersection(2); %Différence D : Point désirée (target) et Yobst
    Alpha = atan2(Y_D_O, X_D_O);
    
    ModuleX = 0.6;
    ModuleY = 1.2;
    %%Affichage des axes du repère
    hold on
    figObstacle4(Obstacle_ID) = quiver(Pt_intersection(1) -  ModuleX*cos(Alpha), Pt_intersection(2) - ModuleX*sin(Alpha),...
        2*ModuleX*cos(Alpha), 2*ModuleX*sin(Alpha),...
        'MaxHeadSize', 0.3, 'Marker','.', 'MarkerSize', 4, 'LineStyle','--','LineWidth',2,'Color', [0.4 0.4 0.4]);%Axe des X, pour le GRIS [0.4 0.4 0.4]
    hold on
    figObstacle5(Obstacle_ID) = quiver(Pt_intersection(1) -  ModuleY*cos(Angle_Cible_AxeYObstacle), Pt_intersection(2) - ModuleY*sin(Angle_Cible_AxeYObstacle),...
        2*ModuleY*cos(Angle_Cible_AxeYObstacle), 2*ModuleY*sin(Angle_Cible_AxeYObstacle),...
        'MaxHeadSize', 0.3, 'Marker','.', 'MarkerSize', 4, 'LineStyle','--','LineWidth',2,'Color', [0.4 0.4 0.4]);%Axe des Y, pour le GRIS
    %     %%%Dessin des tangentes par rapport à la cible
    PointsTangentes=Obstacle.PointsTangentes;
    hold on,
    plot( PointsTangentes(1, 1), PointsTangentes(2, 1), 'o', 'LineWidth', 2, 'Color',[153/255 0 0]);
    PT_1 = ['PT_{1' num2str(GetID(Obstacle)) 'T}'];
    %%text( PointsTangentes(1, 1)+0.1, PointsTangentes(2, 1), PT_1 , 'FontSize', 8, 'LineWidth', 5, 'Color',[1 0 0]);
    hold on
    %%%plot( [PointsTangentes(1, 2), Xd], [PointsTangentes(2, 2), Yd], 'LineStyle','--', 'LineWidth',0.5,'Color',[153/255 0 0]);
    %The plot of the point of the center
    hold on,
    plot( PointsTangentes(1, 2), PointsTangentes(2, 2), 'o', 'LineWidth', 2, 'Color',[153/255 0 0]);
    PT_2 = ['PT_{2' num2str(GetID(Obstacle)) 'T}'];
    %%text( PointsTangentes(1, 2)+0.1, PointsTangentes(2, 2), PT_2, 'FontSize', 8, 'LineWidth', 5, 'Color',[1 0 0]);
end

%=============================================
% Marge autour de l'obstacle    ||
%=============================================
if bMarge
    hold on,
    figObstacle6(Obstacle_ID) = DessinCourbeParallele(GetA(Obstacle), GetB(Obstacle), ...
        GetX(Obstacle), GetY(Obstacle), ...
        GetPhi(Obstacle), GetMarge(Obstacle), 0); %% le dernier paramètre == typeMotif
end

if bID
    hold on
    TextObstacle1(Obstacle_ID) = text(GetX(Obstacle), GetY(Obstacle)+0.1, num2str(GetID(Obstacle)), 'FontSize', 8, 'Color', 'r'); %%Pour le dessin de l'indice de l'obstacle
end

axis([Xmin_axis Xmax_axis Ymin_axis Ymax_axis]);