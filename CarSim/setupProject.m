function setupProject()
%SETUPPROJECT Add the project to the MATLAB path and create data folders.
    root = fileparts(mfilename('fullpath'));
    addpath(root);                          % makes +vehicle, +track, ... visible
    addpath(fullfile(root, 'scripts'));
    for d = ["data/tracks", "data/sessions"]
        folder = fullfile(root, d);
        if ~isfolder(folder), mkdir(folder); end
    end
end
