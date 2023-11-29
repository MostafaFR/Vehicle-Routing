% Joelle AL HAGE 

clear all
close all
global odometer1 odometer2 odometer3 podo  podo1 podo2 podo3 
global pyp gYp x 

load('Xtruth1.mat') %ground truth
load('Xtruth2.mat')
load('Xtruth3.mat')

t=1;

matrice=[];
B=0;

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


podo1=[0.1 0 0 ; 0 0.1 0 ;0 0 0.05];
podo2=podo1;
podo3=podo1;

podo12=zeros(3,3);
podo13=zeros(3,3);
podo23=zeros(3,3);


podo=[podo1 podo12 podo13; podo12 podo2 podo23; podo13 podo23 podo3];
aa(t)=det(podo);

Q=[0.6*10^-5 0 0;0 0.4*10^-5 0;0 0 10^-5];
QQ=0;

for t=2:260

 %fprintf('\ntemps %d\n\n',t)

 %étape de prédiction
 podo1=podo([1:3],[1:3]);
 podo2=podo([4:6],[4:6]);
 podo3=podo([7:9],[7:9]);
    

    A1=[cos(odometer1(3,t-1)) 0 ; sin(odometer1(3,t-1)) 0 ; 0 1];
    B1=[cos(odometer1(3,t-1)) 0; sin(odometer1(3,t-1)) 0;0 1];
    odometer1(:,t)=odometer1(:,t-1)+ A1*[dista1(t-1); omegac1(t-1)];
    FF1= [ 1 0  -dista1(t-1)*sin(odometer1(3,t-1)) ; 0 1 dista1(t-1)*cos(odometer1(3,t-1)) ; 0 0 1];
    podo1= (FF1*podo1*FF1') + B1*QQ*B1'+Q;
    
    % robot2
    A2=[cos(odometer2(3,t-1)) 0 ; sin(odometer2(3,t-1)) 0 ;  0 1];
    B2=[cos(odometer2(3,t-1)) 0; sin(odometer2(3,t-1)) 0;0 1];
    odometer2(:,t)=odometer2(:,t-1)+ A2*[dista2(t-1); omegac2(t-1)]; %randn(1,1)*0.01
    FF2= [ 1 0  -dista2(t-1)*sin(odometer2(3,t-1)) ; 0 1 dista2(t-1)*cos(odometer2(3,t-1)) ; 0 0 1 ];
    podo2= (FF2*podo2*FF2') + B2*QQ*B2'+Q;

     % robot3
    A3=[cos(odometer3(3,t-1)) 0 ; sin(odometer3(3,t-1)) 0 ; 0 1];
    B3=[cos(odometer3(3,t-1)) 0; sin(odometer3(3,t-1)) 0;0 1];
    odometer3(:,t)=odometer3(:,t-1)+ A3*[dista3(t-1); omegac3(t-1)];
    FF3= [ 1 0  -dista3(t-1)*sin(odometer3(3,t-1)) ; 0 1 dista3(t-1)*cos(odometer3(3,t-1)) ; 0 0 1 ];
    podo3= (FF3*podo3*FF3') + B3*QQ*B3'+Q;


x=[odometer1(:,t);odometer2(:,t);odometer3(:,t)]; %vecteur d'état complet

%les intercovariances
podo12=FF1*podo([1:3],[4:6])*FF2'; %podo([1:3],[4:6]) obtenue de l'étape de mise à jour à l'instant d'avant.
podo13=FF1*podo([1:3],[7:9] )*FF3';
podo23=FF2*podo([4:6],[7:9])*FF3';


podo=[podo1 podo12 podo13; podo12' podo2 podo23; podo13' podo23' podo3]; %matrice de covariance prédite (complète)
gYp=inv(podo);
pyp=gYp*x;


%% les mesures
n=6; %le nombre de mesures prise en compte
sgI=zeros(9,9); %initialisation des gains informatinnels
sgi=zeros(9,1);


for j=1:n   
    [H,z,zestime]=robot3_Sy28(j,t); 


%% Etape de correction du filtre informationnel 
 
 R=[0.0078 0.001 0; 0.001 0.0099 0; 0 0 0.011];
 
 gI=H'*(inv(R))*H; %contribution informationnelle d'une mesure sur la matrice informationnelle
 sgI=sgI+gI; %somme des contributions informationnelles
 
 nu=(z-zestime);
 gi=H'*(inv(R))*[nu+H*(inv(gYp)*pyp)]; %contribution informationnelle d'une mesure sur le vecteur informationnel
 sgi=sgi+gi; %somme des contributions informationnelles
         

  end
        
%% Correction
pyo=pyp+sgi;
gYo=gYp+sgI;

x=inv(gYo)*pyo; %vecteur d'état mis à jour
odometer1(:,t)=x([1:3]);
odometer2(:,t)=x([4:6]);
odometer3(:,t)=x([7:9]);
podo=inv(gYo); % matrice de covariance mise à jour

%les régions 3 sigma
  aa1(t)=sqrt(podo(2,2));
  aa2(t)= sqrt(podo(5,5));
  aa3(t)=sqrt(podo(8,8));
  
end

erx1=Xtruth1(:,1)'-odometer1(1,:);
ery1=Xtruth1(:,2)'-odometer1(2,:);
er1=sqrt(erx1.^2+ery1.^2);
fprintf('\ner1= %f\n',mean(er1));

erx2=Xtruth2(:,1)'-odometer2(1,:);
ery2=Xtruth2(:,2)'-odometer2(2,:);
er2=sqrt(erx2.^2+ery2.^2);
fprintf('\ner2= %f\n',mean(er2));

erx3=Xtruth3(:,1)'-odometer3(1,:);
ery3=Xtruth3(:,2)'-odometer3(2,:);
er3=sqrt(erx3.^2+ery3.^2);
fprintf('\ner3= %f\n',mean(er3));

hold on
figure(1);
hold on
plot(aa1(2:t),'b');
plot(-aa1(2:t),'b');
title('the 3 sigma region');
plot(ery1,'r')
hold off


figure(2);
hold on
plot(aa2(2:t),'b'); 
plot(-aa2(2:t),'b');
title('the 3 sigma region');
plot(ery2,'r')
hold off

figure(3);
hold on
plot(aa3(2:t),'b'); 
plot(-aa3(2:t),'b');
title('the 3 sigma region');
plot(ery3,'r')
hold off

figure(4)
plot(odometer1(1,:),odometer1(2,:),'r');
hold on
plot(odometer2(1,:),odometer2(2,:),'b');
plot(odometer3(1,:),odometer3(2,:),'g');
legend('robot1','robot2','robot3');
title('trajectoire');
plot(Xtruth1(:,1),Xtruth1(:,2),':r','linewidth',2);
plot(Xtruth2(:,1),Xtruth2(:,2),':b','linewidth',2);
plot(Xtruth3(:,1),Xtruth3(:,2),':g','linewidth',2);

figure(5)
plot(er1)
hold on
plot(er2)
plot(er3)


