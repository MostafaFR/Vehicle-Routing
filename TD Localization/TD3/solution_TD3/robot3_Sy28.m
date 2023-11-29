function [H,z,zestime]=robot3_Sy28(j,t)

%--------------------------------------------------------------------------
% programme dans le cas où on a 3 robots dans le système
% Trouver les positions relatives de chaque robot par rapport à l'autre
% (mesurée et prédicte);
% Joelle Al Hage
%--------------------------------------------------------------------------

load('X12')
load('X21')
load('X13')
load('X31')
load('X23')
load('X32')

load('angles1.mat'); 
load('angles2.mat');
load('angles3.mat');



yaw2=angles2;
yaw1=angles1;
yaw3=angles3;

global x odometer1 odometer2 odometer3 %variables globales partagées avec le fichier TD3

    if j==1
    
        %fprintf('le robot 1 observe le robot 2 \n');
        xp21(t)=cos(yaw1(t)).*X21(1,t)+sin(yaw1(t)).*X21(2,t); %l'abscisse du robot 2 par rapport au robot 1 ,calculé par rapport au repère 1
        yp21(t)=-sin(yaw1(t)).*X21(1,t)+cos(yaw1(t)).*X21(2,t);
        tetap21(t)=yaw2(t)-yaw1(t);
        z1=[xp21(t);yp21(t);tetap21(t)];
        z=z1;
         
        rot=[cos(odometer1(3,t)) sin(odometer1(3,t))  0;-sin(odometer1(3,t)) cos(odometer1(3,t))  0;0 0 1];
        z1estime=rot*[-eye(3,3) eye(3,3) zeros(3,3)]*x;
        zestime=z1estime;
        a1=[-1 0  (odometer2(2,t))-(odometer1(2,t))  ;0 -1  -(odometer2(1,t))+(odometer1(1,t)) ;0 0 -1 ];
        h21=[a1,eye(3,3),zeros(3,3)];
        H21=rot*h21; %represente la jacobienne de la mesure 
        H=H21;
  
    elseif j==2
        %fprintf('le robot 2 observe le robot 1 \n');
       
        xp12(t)=cos(yaw2(t)).*X12(1,t)+sin(yaw2(t)).*X12(2,t); 
        yp12(t)=-sin(yaw2(t)).*X12(1,t)+cos(yaw2(t)).*X12(2,t);
        tetap12(t)=yaw1(t)-yaw2(t);

        z2=[xp12(t);yp12(t);tetap12(t)]; 
        z=z2;
        
        rot=[cos(odometer2(3,t)) sin(odometer2(3,t)) 0 ; -sin(odometer2(3,t)) cos(odometer2(3,t)) 0 ;0 0 1];
        z2estime=rot*[eye(3,3) -eye(3,3) zeros(3,3)]*x;
        zestime=z2estime;
        
        a2=[-1 0 (odometer1(2,t))-(odometer2(2,t))  ;0 -1 -(odometer1(1,t))+(odometer2(1,t)) ;0 0 -1 ];
        h12=[eye(3,3),a2,zeros(3,3)];
        H12=rot*h12 ;
        H=H12;


            
    elseif j==3
        %fprintf('le robot 1 observe le robot 3 \n');
       
        xp31(t)=cos(yaw1(t)).*X31(1,t)+sin(yaw1(t)).*X31(2,t); 
        yp31(t)=-sin(yaw1(t)).*X31(1,t)+cos(yaw1(t)).*X31(2,t);
        tetap31(t)=yaw3(t)-yaw1(t);
        z3=[xp31(t);yp31(t);tetap31(t)]; 
        z=z3;
        
        rot=[cos(odometer1(3,t)) sin(odometer1(3,t)) 0; -sin(odometer1(3,t)) cos(odometer1(3,t)) 0; 0 0 1];
        z3estime=rot*[-eye(3,3) zeros(3,3) eye(3,3)]*x;
        zestime=z3estime;
        
        a1=[-1 0 (odometer3(2,t))-(odometer1(2,t))  ;0 -1 -(odometer3(1,t))+(odometer1(1,t)) ;0 0 -1 ];
        h31=[a1,zeros(3,3),eye(3,3)];
        H31=rot*h31;
        H=H31;
 
    elseif j==4
        %fprintf('le robot 3 observe le robot 1 \n');

        xp13(t)=cos(yaw3(t)).*X13(1,t)+sin(yaw3(t)).*X13(2,t); 
        yp13(t)=-sin(yaw3(t)).*X13(1,t)+cos(yaw3(t)).*X13(2,t);
        tetap13(t)=yaw1(t)-yaw3(t);
       
        z4=[xp13(t);yp13(t);tetap13(t)]; 
        z=z4;
        
        rot=[cos(odometer3(3,t)) sin(odometer3(3,t)) 0; -sin(odometer3(3,t)) cos(odometer3(3,t)) 0; 0 0 1];
        z4estime=rot*[eye(3,3) zeros(3,3) -eye(3,3)]*x;
        zestime=z4estime;
        
        a3=[-1 0 (odometer1(2,t))-(odometer3(2,t))  ;0 -1  -(odometer1(1,t))+(odometer3(1,t)) ; 0 0 -1 ];
        h31=[eye(3,3),zeros(3,3),a3];
        H31=rot*h31;
        H=H31;
        
        
    elseif j==5
        %fprintf('le robot 2 observe le robot 3 \n'); 

        xp32(t)=cos(yaw2(t)).*X32(1,t)+sin(yaw2(t)).*X32(2,t); 
        yp32(t)=-sin(yaw2(t)).*X32(1,t)+cos(yaw2(t)).*X32(2,t);
        tetap32(t)=yaw3(t)-yaw2(t);

        z5=[xp32(t);yp32(t);tetap32(t)]; 
        z=z5;
        
        rot=[cos(odometer2(3,t)) sin(odometer2(3,t)) 0; -sin(odometer2(3,t)) cos(odometer2(3,t)) 0; 0 0 1];
        z5estime=rot*[zeros(3,3) -eye(3,3) eye(3,3)]*x;
        zestime=z5estime;
        
        a2=[-1 0 (odometer3(2,t))-(odometer2(2,t))  ;0 -1 -(odometer3(1,t))+(odometer2(1,t)) ; 0 0 -1 ];
        h32=[zeros(3,3),a2,eye(3,3)];
        H32=rot*h32;
        H=H32;
        
    elseif j==6
        %fprintf('le robot 3 observe le robot 2 \n');
       
        xp23(t)=cos(yaw3(t)).*X23(1,t)+sin(yaw3(t)).*X23(2,t); 
        yp23(t)=-sin(yaw3(t)).*X23(1,t)+cos(yaw3(t)).*X23(2,t);
        tetap23(t)=yaw2(t)-yaw3(t);

        z6=[xp23(t);yp23(t);tetap23(t)]; 
        z=z6;
        
        rot=[cos(odometer3(3,t)) sin(odometer3(3,t)) 0; -sin(odometer3(3,t)) cos(odometer3(3,t)) 0; 0 0 1];
        z6estime=rot*[zeros(3,3) eye(3,3) -eye(3,3)]*x;
        zestime=z6estime;
    
        a3=[-1 0 (odometer2(2,t))-(odometer3(2,t))  ;0 -1 -(odometer2(1,t))+(odometer3(1,t)) ; 0 0 -1 ];
        h23=[zeros(3,3),eye(3,3),a3];
        H23=rot*h23;
        H=H23;
        
    end
%  
    
  %%  
  
