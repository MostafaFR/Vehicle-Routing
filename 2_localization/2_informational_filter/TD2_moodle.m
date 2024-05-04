clear all
close all
load('dqg.mat')
load('dqd.mat')
load('Xtruth.mat')
load('Ytruth.mat')


l1=[0.5 1.4];
l2=[2 0.6];
l3=[3 1];
l=[l1; l2;l3];

r=(70/2)*10^-3; %rayon des roues droites et gauches
e=230*10^-3; %voie

%pose initiale
odometer(1,1)=0;
odometer(2,1)=0.5;
odometer(3,1)=0;
 

%Qu=[0.5*10^-4 0; 0 0.5*10^-4]; %covariance associée aux bruits du vecteur d'entrée
Q=[0.6*10^-5 0 0;0 0.4*10^-5 0;0 0 10^-5]; %covariance du bruit de modèle 
podo=[1 0 0 ; 0 1 0 ;0 0 0.5]; %valeur initiale de la matrice de covariance P

R=10^-5; %covariance du bruit de mesure

for t=2:260
    
    %Prédiction
    dista(t-1)=r*(dqg(t)+dqd(t))/2; 
    omegac(t-1)=r*(dqd(t)-dqg(t))/e; 
       
    A=[cos(odometer(3,t-1)) 0 ; sin(odometer(3,t-1)) 0 ; 0 1];
    odometer(:,t)=odometer(:,t-1)+ A*[dista(t-1); omegac(t-1)]; % modèle d'évolution
    
    B=[cos(odometer(3,t-1)) 0; sin(odometer(3,t-1)) 0;0 1]; % Jacobienne
    FF= [ 1 0  -dista(t-1)*sin(odometer(3,t-1)) ; 0 1 dista(t-1)*cos(odometer(3,t-1)) ; 0 0 1]; % Jacobienne
    podo= (FF*podo*FF') + Q; % prédiction de la covariance
    X=odometer(:,t); %vecteur d'état prédit
    Y=inv(podo); %matrice informationnelle
    y=Y*X; %vecteur informationnel
    
%     
    %simulate measurements from groundtruth
    rho1(t)=sqrt((Xtruth(t)-l1(1)).^2+(Ytruth(t)-l1(2)).^2);
    rho2(t)=sqrt((Xtruth(t)-l2(1)).^2+(Ytruth(t)-l2(2)).^2);
    rho3(t)=sqrt((Xtruth(t)-l3(1)).^2+(Ytruth(t)-l3(2)).^2);
    z=[rho1(t)+normrnd(0,0.001);rho2(t)+normrnd(0,0.001);rho3(t)+normrnd(0,0.001)]; %mesures bruitées
    
    %initialisation des gains informationnels
    gI=zeros(3,3);
    gi=zeros(3,1);
    
    for i=1:3
        zestime=sqrt((X(1)-l(i,1))^2+(X(2) -l(i,2))^2); %mesure estimée
        H=[(X(1)-l(i,1))/zestime,(X(2)-l(i,2))/zestime,0]; %jacobienne associée à la mesure

        gI=gI+H'*inv(R)*H; % H'*inv(R)*H contribution informationnelle associée à la matrice informationnelle
        nu=z(i)-zestime;
        gi=gi+H'*inv(R)*[nu+H*X]; % H'*inv(R)*[nu+H*X] contribution informtionnelle associée au vecteur informationnel
    end 
    
    Y=Y+gI; % matrice informationnelle mise à jour
    y=y+gi; % vecteur informationnel mis à jour
    
    podo=inv(Y); % matrice de covariance mise à jour
    odometer(:,t)=podo*y; % vecteur d'état mis à jour à l'instant t
    
    sigmax(t)=sqrt(podo(1,1)); %écart type suivant x
    sigmay(t)=sqrt(podo(2,2)); %écart type suivant y

      
end


figure
plot(odometer(1,:),odometer(2,:),'.');
hold on
plot(Xtruth,Ytruth,'.r')
plot(l1(1),l1(2),'*g');
plot(l2(1),l2(2),'*g');
plot(l3(1),l3(2),'*g');

erx=Xtruth'-odometer(1,:);
ery=Ytruth'-odometer(2,:);
er=sqrt(erx.^2+ery.^2);
figure
plot(er)
er=mean(er)

figure
plot(erx)
hold on
plot(3*sigmax(2:260),'r')
plot(-3*sigmax(2:260),'r')
% 
%
figure
hold on
plot(ery)
plot(3*sigmay(2:260),'r')
plot(-3*sigmay(2:260),'r')


