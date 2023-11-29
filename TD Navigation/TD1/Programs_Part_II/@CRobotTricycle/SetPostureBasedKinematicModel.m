%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Université de Technologie de Compiègne (UTC)                       %
%% SY28 :: Département Génie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dernière modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function RobotTricycle=SetPostureBasedKinematicModel(RobotTricycle, LinearVelocity, Gamma)
%% Affectation de la position de l'obstacle
 
TricyclePrameters(); %%Pour récupérer les paramètres intrinsèques au véhicule 
 
global PasEchantillonnageCommande 
 
PostureTricycle = GetPosture(RobotTricycle); %Posture courante [x, y, theta, delta], avant application de la commande
Theta = PostureTricycle(3); 
 
PostureTricycle(1) = PostureTricycle(1) + cos(Theta)*LinearVelocity*PasEchantillonnageCommande;
PostureTricycle(2) = PostureTricycle(2) + sin(Theta)*LinearVelocity*PasEchantillonnageCommande;         
PostureTricycle(3) = PostureTricycle(3) + (tan(Gamma)/lbase)*LinearVelocity*PasEchantillonnageCommande;     
PostureTricycle(4) = Gamma;
 
if abs(PostureTricycle(4))> GammaMax
    PostureTricycle(4) = sign(PostureTricycle(4))*GammaMax;
end
 
RobotTricycle.Posture = PostureTricycle; %%[x y theta Gamma] en suivant le modèle du robot