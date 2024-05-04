function [L_diamond, weights_diamond, L_line, weights_line] = laplacian_matrices(desired_distance)
    % Matrice Laplacienne pour la formation en diamant
    L_diamond = [1 -1 0 0 0;
                 -1 3 -1 -1 0;
                 0 -1 3 -1 -1;
                 0 -1 -1 3 -1;
                 0 0 -1 -1 2];

    % Poids pour les distances entre les robots
    weights_diamond = [0 desired_distance 0 0 0; % Distances du leader aux autres
                       desired_distance 0 0 0 0; % Distances du robot 2 aux autres
                       0 desired_distance 0 desired_distance desired_distance; % Distances du robot 3 aux autres
                       0 desired_distance desired_distance 0 desired_distance; % Distances du robot 4 aux autres
                       0 (2 * sqrt(2) * desired_distance) desired_distance desired_distance 0]; % Distances du robot 5 aux autres

    % Matrice Laplacienne pour la formation en ligne
    L_line = [1 -1 0 0 0
              -1 2 -1 0 0
              0 -1 2 -1 0
              0 0 -1 2 -1
              0 0 0 -1 1];

    % Poids pour les distances entre les robots en formation en ligne
    weights_line = [0, desired_distance, 2 * desired_distance, 3 * desired_distance, 4 * desired_distance
                    desired_distance, 0, desired_distance, 2 * desired_distance, 3 * desired_distance
                    2 * desired_distance, desired_distance, 0, desired_distance, 2 * desired_distance
                    3 * desired_distance, 2 * desired_distance, desired_distance, 0, desired_distance
                    4 * desired_distance, 3 * desired_distance, 2 * desired_distance, desired_distance, 0];
end
