%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Obstacle = SetRepereObstacle(Obstacle)
global Xd Yd
global Exi
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Part to define the reference frame according to the target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DistanceTargetObstacle = sqrt((Xd-Obstacle.Position(1))^2 + (Yd-Obstacle.Position(2))^2);
 
if ( ((Obstacle.A > Exi) || (Obstacle.B > Exi)) && (Obstacle.Marge  > Exi) && (DistanceTargetObstacle > Exi) ) 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%M�thode pour le changement de rep�res
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Y_D_O = Yd - Obstacle.Position(2); %Diff�rence D : Point d�sir�e (Target) et Yobst
X_D_O = Xd - Obstacle.Position(1);
Alpha = atan2(Y_D_O, X_D_O);
 
%Calcul de la matrice de passage du rep�re obstacle (R_O) au rep�re absolu (R_A)
TransformationZ = [cos(Alpha) -sin(Alpha) 0 Obstacle.Position(1)
    sin(Alpha) cos(Alpha) 0 Obstacle.Position(2)
    0 0 1 0
    0 0 0 1];
 
CoordonneesPointsRepereAbsolu = TransformationZ * [0; (Obstacle.A + Obstacle.Marge); 0; 1];
X_Prime = CoordonneesPointsRepereAbsolu(1); %Attention le "prime" ne veut pas dire d�riv�e
Y_Prime = CoordonneesPointsRepereAbsolu(2);
PointsTangentes(1,1) = X_Prime;
PointsTangentes(2,1) = Y_Prime;
 
CoordonneesPointsRepereAbsolu = TransformationZ * [0; -(Obstacle.A + Obstacle.Marge); 0; 1];
X_Prime = CoordonneesPointsRepereAbsolu(1); %Attention le "prime" ne veut pas dire d�riv�e
Y_Prime = CoordonneesPointsRepereAbsolu(2);
PointsTangentes(1,2) = X_Prime;
PointsTangentes(2,2) = Y_Prime;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Obstacle=SetAngleEntreAxesRepere(Obstacle, Alpha+pi/2);
else
Pt_intersection  =  [Xd Yd];
PointsTangentes = [Xd Xd; Yd Yd]; 
AngleEntreAxesRepereObstacle = pi/2;  
Obstacle=SetAngleEntreAxesRepere(Obstacle, pi/2);
end
 
Obstacle=SetCentreRepere(Obstacle, [Obstacle.Position(1) Obstacle.Position(2)]);
Obstacle=SetPointsTangentes(Obstacle, PointsTangentes);