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


function DessinTricycle(Xreel,Yreel, Theta, Gamma, iFigure, Info)
 
TricyclePrameters; %%Function where is all the robot parameters. It is a global function given in the main folder
 
Posture = [Xreel,Yreel, Theta, Gamma];
 
Theta = Posture(3);  % Orientation of the robot (radians)
Gamma =  Posture(4); % Orientation of the front wheel of the robot (gamma)
  
figure(iFigure)   
    % Plot Robot
   figrob = plot([Posture(1)-0.5*wtrack*cos(Theta+pi/2) Posture(1)+0.5*wtrack*cos(Theta+pi/2)],...
        [Posture(2)-0.5*wtrack*sin(Theta++pi/2) Posture(2)+0.5*wtrack*sin(Theta++pi/2)],...
        [Posture(1) Posture(1)+lbase*cos(Theta)],[Posture(2) Posture(2)+lbase*sin(Theta)],...
        [Posture(1)+lbase*cos(Theta)-0.5*rwheel*cos(Theta+Gamma) Posture(1)+lbase*cos(Theta)+0.5*rwheel*cos(Theta+Gamma)],...
        [Posture(2)+lbase*sin(Theta)-0.5*rwheel*sin(Theta+Gamma) Posture(2)+lbase*sin(Theta)+0.5*rwheel*sin(Theta+Gamma)], ...
        'Color',[0.3 0.3 0.3],'linestyle', '-', 'linewidth',1.5);
    
    %% Plot the ellipse which surround the vehicle
    paramRob.Omega = Theta;
    paramRob.Center = [Posture(1)+0.5*lbase*cos(Theta);Posture(2)+0.5*lbase*sin(Theta)];
    eElliprob = plotellipsegral(paramRob);
    figElliprob = plot(eElliprob(1,:), eElliprob(2,:), 'Color', [0.3 0.3 0.3],'linestyle', '-', 'linewidth',1.5);
    
    Rcrrob = lbase/Gamma;
    if Gamma == 0
        Rcrrob = 1e5;
    end


