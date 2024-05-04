function matrice = allouerMatriceNulleAvecBoucles(n, m)
    % Alloue une matrice nulle de taille n x m en utilisant des boucles imbriquées
    matrice = zeros(n, m); % Initialisation pour la taille
    for i = 1:n
        for j = 1:m
            matrice(i, j) = 0;
        end
    end
end
