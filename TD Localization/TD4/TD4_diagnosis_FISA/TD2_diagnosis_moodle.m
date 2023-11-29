clear all
close all
load('dqg.mat')
load('dqd.mat')
load('Xtruth.mat')
load('Ytruth.mat')

l1 = [0.5 1.4];
l2 = [2 0.6];
l3 = [3 1];
l4 =[3.5 1.6];
l5 = [0.1 0.1];
l = [l1; l2; l3; l4; l5];

r = (70/2) * 10 ^ -3; %rayon des roues droites et gauches
e = 230 * 10 ^ -3; %voie

%pose initiale
odometer(1, 1) = 0;
odometer(2, 1) = 0.5;
odometer(3, 1) = 0;

Qu = [0.5 * 10 ^ -4 0; 0 0.5 * 10 ^ -4] * 0; %covariance associ�e aux bruits du vecteur d'entr�e
Q = [0.6 * 10 ^ -5 0 0; 0 0.4 * 10 ^ -5 0; 0 0 10 ^ -5]; %covariance du bruit de mod�le
podo = [1 0 0; 0 1 0; 0 0 0.5]; %valeur initiale de la matrice de covariance P

R = 10 ^ -5; %covariance du bruit de mesure

tfault1 = [20:50];
tfault2 = [20:50, 80:100];
tfault5 = [150:180];

for t = 2:260
    fprintf('\ntemps %d\n\n', t)
    %Pr�diction
    dista(t - 1) = r * (dqg(t) + dqd(t)) / 2;
    omegac(t - 1) = r * (dqd(t) - dqg(t)) / e;
    
    A = [cos(odometer(3, t - 1)) 0; sin(odometer(3, t - 1)) 0; 0 1];
    odometer(:, t) = odometer(:, t - 1) + A * [dista(t - 1); omegac(t - 1)]; % mod�le d'�volution
    
    B = [cos(odometer(3, t - 1)) 0; sin(odometer(3, t - 1)) 0; 0 1]; % Jacobienne
    FF = [1 0 -dista(t - 1) * sin(odometer(3, t - 1)); 0 1 dista(t - 1) * cos(odometer(3, t - 1)); 0 0 1]; % Jacobienne
    podo = (FF * podo * FF') + Q; % pr�diction de la covariance
    X = odometer(:, t); %vecteur d'�tat pr�dit
    Yp = inv(podo); %matrice informationnelle
    yp = Yp * X; %vecteur informationnel
    
    %simulate measurements from groundtruth
    rho1(t) = sqrt((Xtruth(t) - l1(1)) .^ 2 + (Ytruth(t) - l1(2)) .^ 2);
    rho2(t) = sqrt((Xtruth(t) - l2(1)) .^ 2 + (Ytruth(t) - l2(2)) .^ 2);
    rho3(t) = sqrt((Xtruth(t) - l3(1)) .^ 2 + (Ytruth(t) - l3(2)) .^ 2);
    rho4(t) = sqrt((Xtruth(t) - l4(1)) .^ 2 + (Ytruth(t) - l4(2)) .^ 2);
    rho5(t) = sqrt((Xtruth(t) - l5(1)) .^ 2 + (Ytruth(t) - l5(2)) .^ 2);
    
    z = [rho1(t) + normrnd(0, 0.001); rho2(t) + normrnd(0, 0.001); rho3(t) + normrnd(0, 0.001); rho4(t) + normrnd(0, 0.001); rho5(t) + normrnd(0, 0.001)];
    
    %injection de d�fauts
    if find(tfault1 == t)
        z(1) = z(1) + 0.05;
    end
    
    if find(tfault2 == t)
        z(2) = z(2) + 0.05;
    end
    
    if find(tfault5 == t)
        z(5) = z(5) + 0.05;
    end
    
    %initialisation des gains informationnels
    sgI = zeros(3, 3);
    sgi = zeros(3, 1);
    gI = [];
    gi = [];
    th_part = chi2inv(0.9, 3);
    
    %% Etape de correction du filtre informationnel
    
    %
    %% d�tection de d�faut puis isolation
    
    %% correction
    Yu = Yp + sgI; % matrice informationnelle mise � jour
    yu = yp + sgi; % vecteur informationnel mis � jour
    
    podo = inv(Yu); % matrice de covariance mise � jour
    odometer(:, t) = podo * yu; % vecteur d'�tat mis � jour � l'instant t
    sigmax(t) = sqrt(podo(1, 1));
    sigmay(t) = sqrt(podo(2, 2));
    sigmatheta(t) = sqrt(podo(3, 3));
    
end

figure(1)
plot(odometer(1, :), odometer(2, :), '.');
hold on
plot(Xtruth, Ytruth, '.r')
plot(l1(1), l1(2), '*g');
plot(l2(1), l2(2), '*g');
plot(l3(1), l3(2), '*g');
plot(l4(1), l4(2), '*g');
plot(l5(1), l5(2), '*g');

erx = Xtruth' - odometer(1, :);
ery = Ytruth' - odometer(2, :);
er = sqrt(erx .^ 2 + ery .^ 2);
figure(2)
plot(er)
er = mean(er)
