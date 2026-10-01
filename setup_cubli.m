function setup_cubli()
%SETUP_CUBLI Add the project's source folders to the MATLAB path. Run once per
%session, from the project root:
%
%     cd <this folder>
%     setup_cubli
%     run_cubli_clean
%
% The scripts call each other by bare name (build_cubli_clean_cubli calls
% cubli_clean_parameters, and so on), so their folders have to be on the path
% together.
%
% WHY NOT startup.m: startup.m only runs when MATLAB starts IN this folder.
% Open MATLAB anywhere else and it silently does nothing. An explicit function
% that you run is unambiguous.
%
% This file lives in the project root, and MATLAB always has the CURRENT FOLDER
% on the path -- so as long as you start from here, this function is findable
% and everything else follows.

here = fileparts(mfilename('fullpath'));
addpath(here);                                % project root: cubli_root.m lives here

% Add whatever source folders are actually present, so this works both in the
% full development tree and in a trimmed release.
added = strings(1,0);
for d = ["core","builders","experiments","diagnostics","verification"]
    p = fullfile(here,'src',char(d));
    if isfolder(p)
        addpath(p);
        added(end+1) = "src/" + d; %#ok<AGROW>
    end
end

% Every script calls this, so it only speaks the first time.
persistent announced
if isempty(announced), announced = false; end
if ~announced
    announced = true;
    fprintf('Cubli path set up.\n   project root : %s\n   on path      : root, %s\n', ...
        here,strjoin(cellstr(added),', '));
end
end
