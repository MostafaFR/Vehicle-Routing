
% Exo1
%Q1
tic;
matrice1 = rand(1000, 1000); % Matrice de taille 1000x1000
matrice2 = rand(1000, 1000); % Matrice de taille 1000x1000
tempsEcouleCreation = toc;
tic;

%Q2
matrice3 = matrice1 * matrice2;
tempsEcouleMultiplication = toc;
tempsEcoule = tempsEcouleCreation + tempsEcouleMultiplication;

fprintf('Temps nécessaire pour créer deux matrices aléatoires : %f secondes.\n', tempsEcouleCreation);
fprintf('Temps nécessaire pour multiplier deux matrices aléatoires : %f secondes.\n', tempsEcouleMultiplication);
fprintf('Temps nécessaire pour créer et multiplier deux matrices aléatoires : %f secondes.\n', tempsEcoule);

% Equivalent FindDelay
X = [1 2 3];
Y = [0 1 9 9 98 55 44 44 44 44 44];
D = finddelay(X,Y)
fprintf('Le délai est de %d ', D);


% Q3
n = 1000; % Taille de la matrice
m = 1000;

% Mesurer le temps pour la fonction avec boucles
tic;
matriceBoucles = allouerMatriceNulleAvecBoucles(n, m);
tempsBoucles = toc;
fprintf('Temps avec boucles : %f secondes.\n', tempsBoucles);

% Mesurer le temps pour la fonction zeros
tic;
matriceZeros = zeros(n, m);
tempsZeros = toc;
fprintf('Temps avec zeros : %f secondes.\n', tempsZeros);



% Exo2
p = 0.1; % Probabilité de passer du bon état au mauvais état
r = 0.9; % Probabilité de passer du mauvais état au bon état
num_packets = 1000; % Nombre total de paquets à envoyer

% Générer la perte de paquets et l'enregistrer dans un fichier
packet_loss = rand(1, num_packets) < p; % Utilisation d'une distribution aléatoire
dlmwrite('PacketLoss.txt', packet_loss, 'delimiter', '\t');

% Calculer le taux de perte de paquet
packet_loss_rate = sum(packet_loss) / num_packets;
disp(['Le taux de perte de paquet est de ', num2str(packet_loss_rate * 100), '%.']);

