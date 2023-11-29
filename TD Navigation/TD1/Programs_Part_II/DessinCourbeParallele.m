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

function ID_PLOT = DessinCourbesParallele(A, B, X0, Y0, Phi, Marge, typeMotif)
indice = 0;
for angle=0:0.001:(2*pi)
    indice = indice + 1;
   
    X_= (A+Marge)*cos(angle);  
    Y_= (A+Marge)*sin(angle); 
    
    Transformation = ([cos(Phi), -sin(Phi), 0, X0;
                        sin(Phi), cos(Phi),  0, Y0;
                        0, 0, 1, 0;
                        0, 0, 0, 1])*[X_ Y_ 0 1]';
    
    X(indice) = Transformation(1);
    Y(indice) = Transformation(2);
end

if (typeMotif ==1)
    hold on;
    ID_PLOT = plot(X, Y, 'LineWidth',1.5, 'lineStyle','--' ,'Color', [1 0 0]); %%%,
else
    hold on;
    ID_PLOT = plot(X, Y, 'LineWidth',1.5, 'lineStyle','-.' ,'Color', [0.2 0.2 0.2]); %%%,
end