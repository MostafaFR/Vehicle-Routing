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

function [Curvature SommeCuravture CurvatureMax] = GetTrajectoryCurvature(Trajectory)
%%Function to compute the different features linked to trajectory curvature
%%%addpath('../Libraries/linecurvature_version1b');

Curvature = LineCurvature2D(Trajectory(:, 1:2));

NumbrePoints = size(Curvature,1);
SommeCuravture = 0;
CurvatureMax = 0;
for i=1:NumbrePoints
    if (~isnan(Curvature(i)) && ~isinf(Curvature(i))) %%Compute the integral / Sum and the maximum attained curavtre
       SommeCuravture = SommeCuravture + Curvature(i)^2;
       if (abs(Curvature(i)) > CurvatureMax)
          CurvatureMax = abs(Curvature(i));
       end   
    end 
end

