% creer la fonction diagostic
% creer la fonction de calcul de la matrice de covariance

function [r] = diagnostic(gI, gi, yp, Yp, X)
yu = yp+gi;
Yu = Yp+gI;
Xkk = inv(Yu)*yu;
Xkp = X;
r = (Xkk-Xkp)' * Yu * (Xkk-Xkp);

