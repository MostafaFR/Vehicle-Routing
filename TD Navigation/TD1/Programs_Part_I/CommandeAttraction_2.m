%Methode alternative
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Part I : Planning and control of mobile robots                     %
%% Th�or�me de stabilit� de Lyapunov et m�thode des cycles-limites    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function CommandeReelle = CommandAttraction(Donnees)
%Donnees = [Ecart; ThetaTilde; Ex; Ey; ThetaReel];
Ecart = Donnees(1);
ThetaTilde = Donnees(2);
Ex = Donnees(3);
Ey = Donnees(4);
ThetaReel = Donnees(5);

% Param�tres PID
Kp_V = 0.3; % R�duit
Ki_V = 0.05; % R�duit
Kd_V = 0.05;

Kp_W = 0.8; % R�duit
Ki_W = 0.1; % R�duit
Kd_W = 0.1;

% Limitation de la commande int�grale
integral_limit = 1;

persistent prev_error_V integral_V
persistent prev_error_W integral_W

if isempty(prev_error_V)
    prev_error_V = 0;
    integral_V = 0;
end

if isempty(prev_error_W)
    prev_error_W = 0;
    integral_W = 0;
end

% Calcul de l'erreur pour V
error_V = Ecart;

% Calcul de l'erreur pour W
error_W = ThetaTilde;

% Calcul de la commande PID pour V
V = Kp_V * error_V + Ki_V * integral_V + Kd_V * (error_V - prev_error_V);

% Calcul de la commande PID pour W
W = Kp_W * error_W + Ki_W * integral_W + Kd_W * (error_W - prev_error_W);

% Mise � jour des erreurs et des int�grales pour la prochaine it�ration
prev_error_V = error_V;
integral_V = integral_V + error_V;
integral_V = max(min(integral_V, integral_limit), -integral_limit); % Limitation

prev_error_W = error_W;
integral_W = integral_W + error_W;
integral_W = max(min(integral_W, integral_limit), -integral_limit); % Limitation

ValeurFonctionLyapunov = 0.5*(Ex^2+Ey^2);
CommandeReelle = [V, W, ValeurFonctionLyapunov];
end
