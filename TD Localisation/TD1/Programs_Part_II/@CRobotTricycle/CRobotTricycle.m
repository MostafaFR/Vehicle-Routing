%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lounis ADOUANE                                                     %
%% Universit� de Technologie de Compi�gne (UTC)                       %
%% SY28 :: D�partement G�nie Informatique (GI)                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Tutorial Part II : Planning and control of mobile robots           %
%% Cognitive long-term planning based on gLC (global Limit-cycle)     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Derni�re modification le 21/03/2022                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Etudiant :
%% Date :
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function RobotTricycle = CRobotTricycle(Posture)
% Robot class constructor
% Cr�er un Robot avec les param�tres d'entr�e
persistent ID

if nargin == 0
    RobotTricycle.Posture = [0 0 0 0]; %%Posture du robot [x, y, Theta, Gamma]
    RobotTricycle.ID = 0;
    RobotTricycle = class(RobotTricycle,'CRobotTricycle');
else
    RobotTricycle.Posture = Posture;
    if isempty(ID)
        ID = 1;
    end
    RobotTricycle.ID = ID;
    RobotTricycle = class(RobotTricycle,'CRobotTricycle');
    ID =ID+1;
end
