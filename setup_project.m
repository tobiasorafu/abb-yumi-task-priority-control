%% ABB YuMi Task-Priority Control - project setup
% Run this once after cloning/downloading the repository.
% It restores the compressed YUMI_LWR.m robot model and adds the shared
% helper functions to the MATLAB path for the current session.

rootDir = fileparts(mfilename('fullpath'));
commonDir = fullfile(rootDir,'common');
modelFile = fullfile(commonDir,'YUMI_LWR.m');
compressedModel = fullfile(commonDir,'YUMI_LWR.m.gz');

if ~isfile(modelFile)
    if ~isfile(compressedModel)
        error('YUMI_LWR.m.gz was not found in the common folder.');
    end
    gunzip(compressedModel, commonDir);
    fprintf('Restored common/YUMI_LWR.m\n');
end

addpath(commonDir);
fprintf('Project setup complete. Open a folder under scenarios/ and run main.m.\n');
