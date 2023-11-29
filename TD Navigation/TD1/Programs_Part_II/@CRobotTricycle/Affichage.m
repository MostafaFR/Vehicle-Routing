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

function Affichage(RobotTricycle, iFigure)
 
global ChampPerceptionRobot
TricyclePrameters(); %%Function where is all the robot parameters, It a global function given in the main folder
 
Posture = GetPosture(RobotTricycle);
Theta =  Posture(3);  % Orientation of the robot (radians)
Gamma =  Posture(4); % Orientation of the front wheel of the robot (gamma)
%%%%
 
global Xmin_axis Xmax_axis Ymin_axis Ymax_axis
axis([Xmin_axis Xmax_axis Ymin_axis Ymax_axis])
 
persistent semaphore figrob figElliprob figRcrrob figRcrrobl figChampPerception %%Initialement, elle est définie à [] par défaut
 
 
%% Plot
if ~isempty(semaphore)
    delete(figrob)
    delete(figElliprob)
    delete(figRcrrob)
    delete(figRcrrobl)
    delete(figChampPerception)
end
semaphore = 1;
 
figure(iFigure)
% Plot Robot
figrob = plot([Posture(1)-0.5*wtrack*cos(Theta+pi/2) Posture(1)+0.5*wtrack*cos(Theta+pi/2)],...
    [Posture(2)-0.5*wtrack*sin(Theta++pi/2) Posture(2)+0.5*wtrack*sin(Theta++pi/2)],...
    [Posture(1) Posture(1)+lbase*cos(Theta)],[Posture(2) Posture(2)+lbase*sin(Theta)],...
    [Posture(1)+lbase*cos(Theta)-0.5*rwheel*cos(Theta+Gamma) Posture(1)+lbase*cos(Theta)+0.5*rwheel*cos(Theta+Gamma)],...
    [Posture(2)+lbase*sin(Theta)-0.5*rwheel*sin(Theta+Gamma) Posture(2)+lbase*sin(Theta)+0.5*rwheel*sin(Theta+Gamma)],'Color',[0.4 0 0],'linewidth',3);
 
%% Plot the ellipse which surround the vehicle
paramRob.Omega = Theta;
paramRob.Center = [Posture(1)+0.5*lbase*cos(Theta);Posture(2)+0.5*lbase*sin(Theta)];
eElliprob = plotellipsegral(paramRob);
figElliprob = plot(eElliprob(1,:), eElliprob(2,:), 'b','linewidth',1);
 
Rcrrob = lbase/Gamma;
if Gamma == 0
    Rcrrob = 1e5;
end
%%Plot of the Instantaneous Center of Rotation of the robot
figRcrrob = plot(Posture(1)+Rcrrob*cos(Theta+pi/2),Posture(2)+Rcrrob*sin(Theta+pi/2),'o','Color',[1 0 0]);
figRcrrobl= plot([Posture(1) Posture(1)+Rcrrob*cos(Theta+pi/2)],...
    [Posture(2) Posture(2)+Rcrrob*sin(Theta+pi/2)],'Color',[0 0 0]);
 
bChampPerception = 1;
if bChampPerception
    hold on,
    indice = 0;
    for angle=0:0.001:(2*pi)
        indice = indice + 1;
        X(indice)= Posture(1) + ChampPerceptionRobot*cos(angle);
        Y(indice)= Posture(2) + ChampPerceptionRobot*sin(angle);
    end
    
    typeMotif = 2;
    if (typeMotif ==1)
        hold on;
        figChampPerception = plot(X, Y, 'LineWidth',1.5, 'lineStyle','--' ,'Color', [1 0 0]); %%%,
    else
        hold on;
        figChampPerception =plot(X, Y, 'LineWidth',1.5, 'lineStyle','-.' ,'Color', 'magenta'); %%%,
    end
end