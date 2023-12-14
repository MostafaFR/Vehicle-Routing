function [L_diamond, weights_diamond, L_line, weights_line] = laplacian_matrices(d)
    % Matrice Laplacienne pour la formation en diamant
    L_diamond = [1 -1 0 0 0;
                 -1 3 -1 -1 0;
                 0 -1 3 -1 -1;
                 0 -1 -1 3 -1;
                 0 0 -1 -1 2];

    % Poids pour les distances entre les robots
    weights_diamond = [0 d 0 0 0; % Distances du leader aux autres
                       d 0 d d 0; % Distances du robot 2 aux autres
                       0 d 0 d d; % Distances du robot 3 aux autres
                       0 d d 0 d; % Distances du robot 4 aux autres
                       0 0 d d 0]; % Distances du robot 5 aux autres

    % Matrice Laplacienne pour la formation en ligne
    L_line = [1 -1 0 0 0
              -1 2 -1 0 0
              0 -1 2 -1 0
              0 0 -1 2 -1
              0 0 0 -1 1];

    % Poids pour les distances entre les robots en formation en ligne
    weights_line = [0 (d/3) 0 0 0; % Distances du leader aux autres
                    (d/3) 0 (d/3) 0 0; % Distances du robot 2 aux autres
                    0 (d/3) 0 (d/3) 0; % Distances du robot 3 aux autres
                    0 0 (d/3) 0 (d/3); % Distances du robot 4 aux autres
                    0 0 2*(d/3) (d/3) 0]; % Distances du robot 5 aux autres
end
