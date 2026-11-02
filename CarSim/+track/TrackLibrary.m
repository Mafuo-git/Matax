classdef TrackLibrary
    %TRACKLIBRARY Lists and loads the circuits stored in data/tracks/*.mat.
    methods (Static)
        function folder = defaultFolder()
            root = fileparts(fileparts(mfilename('fullpath')));   % +track -> project root
            folder = string(fullfile(root, 'data', 'tracks'));
        end

        function names = list(folder)
            if nargin < 1, folder = track.TrackLibrary.defaultFolder(); end
            files = dir(fullfile(folder, '*.mat'));
            names = sort(erase(string({files.name}), ".mat"));
        end

        function trk = load(name, folder)
            if nargin < 2, folder = track.TrackLibrary.defaultFolder(); end
            trk = track.Track.fromFile(fullfile(folder, name + ".mat"));
        end
    end
end
