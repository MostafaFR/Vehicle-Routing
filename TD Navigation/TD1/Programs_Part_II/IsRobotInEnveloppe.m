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

function Response = IsRobotInEnveloppe(X, Y, Center, A, B, Phi, Marge)
%%Response == 1 (vrai) ou 0 (faux)

%%Transforamtion entre repères
DecalX=X-Center(1);
DecalY=Y-Center(2);
%%Rotation selon Z
X_prime=cos(Phi)*DecalX + sin(Phi)*DecalY;
Y_prime=-sin(Phi)*DecalX + cos(Phi)*DecalY;
if (((X_prime^2)/((A+Marge)^2) + (Y_prime^2)/((B+Marge)^2)) <= 1)
    Response = 1;
    %plot(Xr, Yr, 'o')
else
    Response = 0;
    %plot(Xr, Yr, '+')
end