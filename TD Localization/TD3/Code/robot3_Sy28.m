function [H,z,zestime]=robot3_Sy28() % entrée à remplir

%--------------------------------------------------------------------------
% programme dans le cas où on a 3 robots dans le système
% Calculez: z zestime et H
%--------------------------------------------------------------------------

load('X12')%pose du robot 1 par rapport au robot 2 exprimé dans le repère global
load('X21')
load('X13')
load('X31')
load('X23')
load('X32')

load('angles1.mat'); %yaw obtenu à partir du gyro
load('angles2.mat');
load('angles3.mat');


    if j==1 
           
        fprintf('le robot 1 observe le robot 2 \n');
        %xp21(t)= l'abscisse du robot 2 par rapport au robot 1,  calculé par rapport au repère 1
        %yp21(t)= l'ordonnée du robot 2 par rapport au robot 1, calculée par rapport au repère 1
        %tetap21(t)=orientation relative
        %z=[xp21(t);yp21(t);tetap21(t)]; %la mesure
        

        % expression de zestime
        % calcul de la jacobienne
  
    elseif j==2
        fprintf('le robot 2 observe le robot 1 \n');
       

            
    elseif j==3
        %fprintf('le robot 1 observe le robot 3 \n');
       
        
 
    elseif j==4
        %fprintf('le robot 3 observe le robot 1 \n');

      
        
        
    elseif j==5
        %fprintf('le robot 2 observe le robot 3 \n'); 

      
        
    elseif j==6
        %fprintf('le robot 3 observe le robot 2 \n');
       
        
    end
%  
    
  %%  
  
