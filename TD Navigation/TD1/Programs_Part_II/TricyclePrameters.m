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

global wtrobot lrobot wrobot wtrack lbase MinTurningRadius rwheel vmax wmax GammaMax Rrobot 
 
%%%%Vipalb dimensions 
DimensionModulation = 0.1;           % Constant factor to reduce the vehicle dimension / max velcoity, etc.
wtrobot = 650*DimensionModulation;   % Weight of the robot (kg)
lrobot = 1.96*DimensionModulation;   % Length of the robot (m)
wrobot = 1.27*DimensionModulation;   % Width of the robot (m)
 
wtrack = 1.20 *DimensionModulation;  % Track width (m) --> may be only for plot!
lbase =  1.20 *DimensionModulation;  % Wheel base (m) --> used for the kinematic model of the robot
%MinTurningRadius = 3.8*DimensionModulation;   % Minimum turning radius (m)
MinTurningRadius = 0.001;   % Minimum turning radius (m)
rwheel =  0.5*DimensionModulation;    % Radius of the wheel (m)
%%%
 
% Optimal ellipse parameters of the vehicle
paramRob.a = 1.3859*DimensionModulation; %%Big radius
paramRob.b = 0.8980*DimensionModulation; %%Small radius
 
%%
Rrobot = paramRob.a; %%%%Rayon Max du cercle qui entoure le robot en [m]
 
%% Data of the Motor (Speed & steering lock angle) (Characteristcs)
%vmax = 1.5*DimensionModulation;          % Maximum linear velocity of the robot (m/s)
vmax = 0.5;
%vmax = 0.1*DimensionModulation;  % Maximum linear velocity of the robot (m/s)
wmax = vmax/MinTurningRadius;     % Maximum angular velocity of the robot (rad/s)
 
%speedresol = 0.1;                % Speed resolution (m/s)
GammaMax = 45*pi/180;             % Steering lock angle (radians)
 
% Initial ellipse parameters (which surround the robot)
% paramEllip.a = 0;
% paramEllip.b = 0;
% paramEllip.Omega = 0;
% paramEllip.Center = [0;0];