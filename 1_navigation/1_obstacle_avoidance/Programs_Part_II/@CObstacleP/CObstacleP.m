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


function Obstacle = CObstacleP(Position, A, B, Phi, Marge, Vitesse, Xrobot, Yrobot)
% CObstacle class constructor
persistent ID
 
if nargin == 0
    Obstacle.Position = [0 0];
    Obstacle.Vitesse = [0 0];
    Obstacle.A = 0;
    Obstacle.B = 0;
    Obstacle.Phi = 0;
    Obstacle.Marge = 0;            
    Obstacle.ID = 0;
    Obstacle.CentreRepere = [0 0]; 
    Obstacle.AngleEntreAxesRepereObstacle = 0;
    Obstacle.PointsTangentes = [0 0; 0 0]; %[X(I(1)), X(I(i)); Y(I(1)), Y(I(i))]; %Est calculé dans la fonction SetRepereObstacle / à la cible
    Obstacle.PointsTangentesRobot = [0 0; 0 0]; %[X(I(1)), X(I(i)); Y(I(1)), Y(I(i))]; %Est calculé dans la fonction SetRepereObstacle / au robot
    Obstacle.SignePositionRelativeXRobot = 0; %%% Correspond à la position relative du robot par rapport au repère lié à l'obstacle 
    Obstacle.SignePositionRelativeYRobot = 0;
    Obstacle.iMinuteurEscape = 0; %%Minuteur pour sortir ou non du champ d'attraction de l'obstacle
    Obstacle = class(Obstacle,'CObstacleP');
else
    Obstacle.Position = Position;
    Obstacle.Vitesse = Vitesse;
    Obstacle.A = A;
    Obstacle.B = B;
    Obstacle.Phi = Phi;
    Obstacle.Marge = Marge;
    if isempty(ID)
        ID = 1;
    end
    Obstacle.ID = ID;
    Obstacle.CentreRepere = [0 0]; %Va être calculé par la suite dans SetRepereObstacle
    Obstacle.AngleEntreAxesRepereObstacle = 0; %Va être calculé par la suite dans SetRepereObstacle
    Obstacle.PointsTangentes = [0 0; 0 0]; %Va être calculé par la suite dans SetRepereObstacle
    Obstacle.PointsTangentesRobot = [0 0; 0 0]; %Va être calculé par la suite dans SetRepereObstacle
    Obstacle.SignePositionRelativeXRobot = 0; %%% Va être calculé dans SetSignePositionRelativeRobot
    Obstacle.SignePositionRelativeYRobot = 0; %%% //
    Obstacle.iMinuteurEscape = 0; %%Minuteur pour sortir ou non du champ d'attraction de l'obstacle
    Obstacle = class(Obstacle, 'CObstacleP'); 
    Obstacle = SetRepereObstacle(Obstacle);
    ID =ID+1;
end