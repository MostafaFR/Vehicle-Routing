
clear all
close all


load('Xtruth1.mat') %ground truth
load('Xtruth2.mat')
load('Xtruth3.mat')

t=1;

%Données odométriques
load('dista1.mat')
load('dista2.mat')
load('dista3.mat')

load('omegac1.mat')
load('omegac2.mat')
load('omegac3.mat')

odometer1(1,1)=0;
odometer1(2,1)=0;
odometer1(3,1)=0;

odometer2(1,1)=0;
odometer2(2,1)=-0.5;
odometer2(3,1)=0;

odometer3(1,1)=0;
odometer3(2,1)=0.5;
odometer3(3,1)=0;


x=[odometer1(:,t);odometer2(:,t);odometer3(:,t)];


podo1=[0.1 0 0 ; 0 0.1 0 ;0 0 0.05]; %covariance initiale associée au robot 1
podo2=podo1;
podo3=podo1;

podo12=zeros(3,3);
podo13=zeros(3,3);
podo23=zeros(3,3);

podo=[podo1 podo12 podo13; podo12 podo2 podo23; podo13 podo23 podo3]; %covariance du système constitué de 3 robots

Q=[0.6*10^-5 0 0;0 0.4*10^-5 0;0 0 10^-5]; %covariance du bruit de modèle
R=[0.0078 0.001 0; 0.001 0.0099 0; 0 0 0.011]; %covariance associée à la mesure de la pose relative

for t=2:260

    %robot 1
    
    % robot2
    
    % robot3
    



%% étapes de mise à jour (somme des contributions informationnelles)

 

         

  
end

