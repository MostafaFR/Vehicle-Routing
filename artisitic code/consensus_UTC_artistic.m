close all;
clear all;
clc;


Db = eye(8);
Ab=[0 0 0 0 0 0 0 1;
    1 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    0 0 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 1 0];
Lb = Db - Ab;

Abis=[
    0 0 0 0 0 0 1 1;
    1 0 0 0 0 0 0 1;
    1 1 0 0 0 0 0 0;
    0 1 1 0 0 0 0 0;
    0 0 1 1 0 0 0 0;
    0 0 0 1 1 0 0 0;
    0 0 0 0 1 1 0 0;
    0 0 0 0 0 1 1 0]

Dbis=2*eye(8);
Lbis = Dbis - Abis;

B = [0 0 0 0 0 0 0 0].';

Te = 0.1;

[Ae, Be] = c2d(-Lb, B, Te);
% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 100;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
Xi = [0 1.75 2 1.75 0 -1.75 -2 -1.75]';
Yi = [2 1.75 0 -1.75 -2 -1.75 0 1.75]';

Temps = linspace(0, iterations*Te, iterations+1);
X = zeros(iterations, 8);
Y = zeros(iterations, 8);

g = 1;
signe = 1;
nb_changement = 1;

%Iterate for the previously specified number of iterations
for t = 1:iterations  
    if mod(t, floor(25*nb_changement)) == 0 
        t
        Lb = Lb';
        g = -g;
        [Ae, Be] = c2d(-g*Lb, B, Te);
        % nb_changement = nb_changement * 1.2;
    end
    %% Algorithm
    X(t,:) = Xi';
    Y(t,:) = Yi';
    
    Xi = Ae * Xi; 
    Yi = Ae * Yi;
    
end

X1 = X(:,1);
X2 = X(:,2);
X3 = X(:,3);
X4 = X(:,4);
X5 = X(:,5);
X6 = X(:,6);
X7 = X(:,7);
X8 = X(:,8);
Y1 = Y(:,1);
Y2 = Y(:,2);
Y3 = Y(:,3);
Y4 = Y(:,4);
Y5 = Y(:,5);
Y6 = Y(:,6);
Y7 = Y(:,7);
Y8 = Y(:,8);
plot(X1, Y1, X2, Y2, X3, Y3, X4, Y4, X5, Y5, X6, Y6, X7, Y7, X8, Y8)
axis equal
pbaspect([2 2 1])