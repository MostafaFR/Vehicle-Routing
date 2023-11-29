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


function ThetaTilde=SoustractionAnglesAtan2(ThetaC, Theta)
%Fonction pour faire de la soustraction d'angles afin d'avoir toujours un
%angle exprimé dans [-pi pi]
 
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Conditionnement des angles
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
if le(ThetaC, 0) %ThetaC<=0 %Exprimer l'angle toujours en positif (0°->2pi)
    ValeurT = mod(ThetaC, -2*pi); %%Pour être sûr que l'angle est bien compris entre [0 et -2pi] 
    ThetaC=(2*pi)+ValeurT;
end
%%%Test si ThetaC est plus grand que 2pi 
if ge(ThetaC, 2*pi) %ThetaC<=0 %Exprimer l'angle toujours en positif (0°->2pi)
    ThetaC = mod(ThetaC, 2*pi); %%Pour être sûr que l'angle est bien compris entre [0 et 2pi] 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
if le(Theta, 0)% Si Theta<=0 %Exprimer l'angle toujours en positif (0°->2pi)
    ValeurT = mod(Theta, -2*pi); %%Pour être sûr que l'angle est bien compris entre [0 et -2pi] 
    Theta=(2*pi)+ValeurT;
end
%%%Test si Theta est plus grand que 2pi 
if ge(Theta, 2*pi) %ThetaC<=0 %Exprimer l'angle toujours en positif (0°->2pi)
    Theta = mod(Theta, 2*pi); %%Pour être sûr que l'angle est bien compris entre [0 et 2pi] 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
 
ThetaTilde = ThetaC - Theta;  %%%Si ThetaC et Theta appartiennent à [0 2pi] 
                              %%%alors leur différence appartient alors à [-2pi +2pi]
if ge(ThetaTilde,pi) %Si ThetaTilde >= pi %Exprimer l'angle (compris entre -pi et pi) 
                     %alors l'angle doit être compris entre [-pi et 0]   
    %ThetaTilde = mod(ThetaTilde, pi); %%Pour être sûr que l'angle est bien compris entre [0 et pi] 
    ThetaTilde = -( (2*pi)-ThetaTilde );
end
 
if le(ThetaTilde,-pi) %Si ThetaTilde <= pi %Exprimer l'angle (compris entre -pi et pi)
                      %alors l'angle doit être compris entre [0 et pi]   
    %ThetaTilde = mod(ThetaTilde, -pi); %%Pour être sûr que l'angle est bien compris entre [0 et pi]
    ThetaTilde = (2*pi)+ThetaTilde;
end
 
if (le(ThetaTilde, -pi) || gt(ThetaTilde, pi));
disp('ATTENTION ANGLES THETA_Tilde');
ThetaC
Theta
ThetaTilde
end