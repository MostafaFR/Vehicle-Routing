close all;
clear all;
clc;

Da = [3 0 0 0;
          0 3 0 0;
          0 0 3 0;
          0 0 0 3];
Aa= ones(4) - eye(4);
La = Da - Aa

Db = eye(4);
Ab = [0 0 0 1;
    1 0 0 0;
    0 1 0 0;
    0 0 1 0];
Lb = Db - Ab;

Dc=[3 0 0 0;
    0 3 0 0;
    0 0 3 0;
    0 0 0 5];

Ac=[0 1 1 1;
    1 0 1 1;
    1 1 0 1;
    3 1 1 0];
Lc = Dc - Ac;


B = [0 0 0 0].';

Te = 0.1;

[Ae, Be] = c2d(-Ld, B, Te)
% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1000;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
Xi = [0 2 0 -2]';
Yi = [2 0 -2 0]';

Temps = linspace(0, iterations*Te, iterations+1);
X = zeros(iterations, 4);
Y = zeros(iterations, 4);

% p1 = zeros(iterations, 2);
% p2 = zeros(iterations, 2);
% p3 = zeros(iterations, 2);
% p4 = zeros(iterations, 2);

%Iterate for the previously specified number of iterations
for t = 1:iterations        
    %% Algorithm
%     p1(t,:) = [Xi(1) Yi(1)];
%     p2(t,:) = [Xi(2) Yi(2)];
%     p3(t,:) = [Xi(3) Yi(3)];
%     p4(t,:) = [Xi(4) Yi(4)];
    X(t,:) = Xi';
    Y(t,:) = Yi';
    
    Xi = Ae * Xi; 
    Yi = Ae * Yi;
    
end

X1 = X(:,1);
X2 = X(:,2);
X3 = X(:,3);
X4 = X(:,4);
Y1 = Y(:,1);
Y2 = Y(:,2);
Y3 = Y(:,3);
Y4 = Y(:,4);
plot(X1, Y1, X2, Y2, X3, Y3, X4, Y4)


    