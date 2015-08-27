
function setup_imostoolbox_aims(varargin)
%%
disp('Set paths for IMOS toolbox processing and beyond, including');
disp('AIMS imos-datatools, AIMS easyplot, and OpenEarthTools.');

%pc
%baseDIR='C:\AIMS';
%pc-dev
baseDIR='D:\Projects\aims-gitlab';
% location of clones of AODN github repositories
AODNbaseDir='D:\Projects\github\aodn';
%hpc
%baseDIR='/export/share/ocean/AIMS';

% path to OpenEarthTools folder
OETdir=fullfile(baseDIR,'aims-matlab','OpenEarthTools');
% path to AIMS imos-datatools, needed for getAllFiles etc
IDTdir=fullfile(baseDIR,'imos-datatools');
% path to AIMS Easyplot
EPdir=fullfile(baseDIR,'easyplot');
% path to IMOS user code library
IUCLdir=fullfile(AODNbaseDir,'imos-user-code-library','MATLAB_R2011');

% path to AIMS matlab (to find aims_proc and friends)
AMdir=fullfile(baseDIR,'aims-matlab');

%
defaultPath = '';
defaultAddOET=true; % add openearth tools
defaultAddNCT=true; % add nctoolbox, used by imos user code library

%% user should not need to edit anything further
[imosToolboxDir, name, ext] = fileparts(mfilename('fullpath'));

p=inputParser;
%addParamValue(p,'path',defaultPath,@isstring);
if verLessThan('matlab','8.2')
    addParamValue(p,'addOET',defaultAddOET,@(x) assert(islogical(x)));
    addParamValue(p,'addNCT',defaultAddNCT,@(x) assert(islogical(x)));
else
    addParameter(p,'addOET',defaultAddOET,@(x) assert(islogical(x)));
    addParameter(p,'addNCT',defaultAddNCT,@(x) assert(islogical(x)));
end

p.KeepUnmatched = true;
parse(p, varargin{:});

%% add paths, order is delibrate
% OpenEarth toolbox
if p.Results.addOET
    if ~exist('oetsettings','file')
        disp('Adding OpenEarthTools, please wait ...');
        run(fullfile(OETdir,'oetsettings.m'))
    else
        [theDir, ~, ~] = fileparts(which('oetsettings'));
        disp('There appears to an OpenEarthTools already on your path.');
        disp(['At : ' theDir]);
    end
end

% nctoolbox
if p.Results.addNCT
    if ~exist('setup_nctoolbox','file')
    disp('Adding nctoolbox, please wait ...');
    addpath(fullfile(AMdir,'nctoolbox'));
    setup_nctoolbox;
    else
        [theDir, ~, ~] = fileparts(which('setup_nctoolbox'));
        disp('There appears to an OpenEarthTools already on your path.');
        disp(['At : ' theDir]);
    end
end

%% add AIMS imos-datatools paths
reAddPaths(IDTdir,'AIMS imos-datatools',true);

%% add IMOS Toolbox paths
reAddPaths(imosToolboxDir,'IMOS toolbox',true);

%% add IMOS user code library paths
reAddPaths(IUCLdir,'IMOS user code library',true);

%% add AIMS easyplot paths
reAddPaths(EPdir,'AIMS easyplot',true);

%% add AIMS aims_proc paths
% aims_proc has aims_vpca etc
reAddPaths(fullfile(AMdir,'aims_proc'),'AIMS aims_proc',true);
% nan safe routines used by aims_vpca
reAddPaths(fullfile(AMdir,'NaN-2.8.1'),'AIMS NaN toolbox',true);
% IGRF
reAddPaths(fullfile(AMdir,'igrf'),'IGRF toolbox',true);

%% add java paths
%disp('Adding IMOS java jars, please wait ...');
%disp(['IMOS toolbox java path : ' fullfile(imosToolboxDir, 'Java-dev')]);
%warning off
%try
%    astr=computer;
%    % IMOS Java
%    %addjars(fullfile(imosToolboxDir,'Java'));
%    % IMOS sbs Java-dev
%    addjars(fullfile(imosToolboxDir, 'Java-dev'));
%    %addjars(fullfile(data_tb_home,'util','ssh2_v2_m1','ganymed-ssh2-m1'));
%catch me
%    ex = MException('IMOS:IMOSTOOLBOX', 'Failed to setup the Java classpath');
%    ex.throw;
%end
%warning on;
end

%%
function reAddPaths(topDir,messageStr,atBeginning)

disp(['Adding paths for ' messageStr ', please wait...']);
gp=genpath_clean(topDir);
disp('  Removing any existing paths.')
rempath(gp);
disp('  Adding new paths');
if atBeginning
    addpath(gp,'-begin');
else
    addpath(gp);
end

end

%%
function rempath(gp)
% from a genpath path string, remove on paths that are currently in the
% matlab path

ss=strsplit(gp,';');
pp=strsplit(path,';');
ii=ismember(ss,pp);
ss=ss(ii); %list with only directories that are currently on matlab path
if ~isempty(ss)
    thePath=sprintf(['%s' pathsep],ss{:}); %make string seperated by pathsep
    if thePath(end)==pathsep % remove last not needed pathsep
        thePath(end)=[];
    end
    rmpath(thePath);
end

end

%%
function thePath = genpath_clean( topDir )
%genpath_clean generate a path without .svn, .git etc.
%   generate a path without .svn, .git etc. Based on idea from OpenEarthTools

b=genpath(topDir);
s = strread(b, '%s','delimiter', pathsep);  % read path as cell
rpattern='(\.svn|\.git|\.hg)';
ii=cellfun(@isempty,regexp(s,rpattern,'match','once'));
s=s(ii); %cell array without .git etc
thePath=sprintf(['%s' pathsep],s{:}); %make string seperated by pathsep
if thePath(end)==pathsep % remove last not needed pathsep
    thePath(end)=[];
end

end

%%%%%%%%%%

function addjars(dirName)

pattern = 'jar';

dirData = dir(dirName);      % Get the data for the current directory
dirIndex = [dirData.isdir];  % Find the index for directories
fileList = {dirData(~dirIndex).name}';  % Get a list of the files

if ~isempty(fileList)
    % Prepend path to files
    fileList = cellfun(@(x) fullfile(dirName,x),...
        fileList,'UniformOutput',false);
    matchstart = regexp(fileList, pattern);
    fileList = fileList(~cellfun(@isempty, matchstart));
end

% Get a list of the subdirectories
subDirs = {dirData(dirIndex).name};
% Find index of subdirectories that are not '.' or '..'
validIndex = ~ismember(subDirs,{'.','..'});

for iDir = find(validIndex)                  % Loop over valid subdirectories
    nextDir = fullfile(dirName,subDirs{iDir});    % Get the subdirectory path
    fileList = [fileList; getAllFiles(nextDir,pattern)];  % Recursively call getAllFiles
end

javaaddpath(fileList);

end