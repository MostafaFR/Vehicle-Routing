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

function Position = PositionRobotDansRepereObstacle(x, y, AngleEntreAxesRepereObstacle, Pt_intersection)

    global Xd Yd

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%Premi�re transformation par rapport � un rep�re orthogonal et orthonorm�
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X_D_O = Xd - Pt_intersection(1);
    Y_D_O = Yd - Pt_intersection(2); %Diff�rence D : Point d�sir�e (target) et Yobst
    Alpha = atan2(Y_D_O, X_D_O);

    %Calcul de la matrice de passage du rep�re obstacle (R_O) au rep�re absolu (R_A)
    T_O_A = [cos(Alpha) -sin(Alpha) 0 Pt_intersection(1)
             sin(Alpha) cos(Alpha) 0 Pt_intersection(2)
             0 0 1 0
             0 0 0 1];
    CoordonneeRepereObstacleOrthonorme = T_O_A \ [x; y; 0; 1];

    %%Pour introduire que le rep�re est non orthogonal
    x_ = CoordonneeRepereObstacleOrthonorme(1);
    y_ = CoordonneeRepereObstacleOrthonorme(2);
    %%
    Angle = AngleEntreAxesRepereObstacle - Alpha; %Puisque les deux angles sont donn�s par rapport au rep�re absolu
    %%Pour �viter l'ind�termination
    if ((abs(Angle) <= 0.0001) || (abs(abs(Angle) - pi) <= 0.0001))
        dis('Attention incompatibilit� rep�re') %juste pour mentionner qu'il y a une ind�termination
        Position = [x_ y_];
    else
        %%Transformation dans rep�re non orthonorm�
        x_ = (x_ - y_ * (cos(Angle) / sin(Angle)));
        y_ = y_ / sin(Angle);
        Position = [x_ y_];
    end
