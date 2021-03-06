function sample_data = awacParse( filename, mode )
%AWACPARSE Parses ADCP data from a raw Nortek AWAC binary (.wpr) file. If
% processed wave data files (.whd and .wap) are present, these are also 
% parsed.
%
% Parses a raw binary file from a Nortek AWAC ADCP. If processed wave data
% files (.whd and .wap) are present, these are also parsed, to provide wave
% data. Wave data is not read from the raw binary files, as the raw binary
% only contains raw wave data. The Nortek software performs a significant
% amount of processing on this raw wave data to provide standared wave
% metrics such as significant wave height, period, etc.
%
% If wave data is present, it is returned as a separate sample_data struct, 
% due to the fact that the timestamps for wave data are significantly 
% different from the profile and sensor data.
%
% Inputs:
%   filename    - Cell array containing the name of the raw AWAC file 
%                 to parse.
%   mode        - Toolbox data type mode ('profile' or 'timeSeries').
% 
% Outputs:
%   sample_data - Struct containing sample data; If wave data is present, 
%                 this will be a cell array of two structs.
%
% Author: 		Paul McCarthy <paul.mccarthy@csiro.au>
% Contributor: 	Guillaume Galibert <guillaume.galibert@utas.edu.au>
%

%
% Copyright (c) 2009, eMarine Information Infrastructure (eMII) and Integrated 
% Marine Observing System (IMOS).
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are met:
% 
%     * Redistributions of source code must retain the above copyright notice, 
%       this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright 
%       notice, this list of conditions and the following disclaimer in the 
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of the eMII/IMOS nor the names of its contributors 
%       may be used to endorse or promote products derived from this software 
%       without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.
%
error(nargchk(1, 2, nargin));

if ~iscellstr(filename), error('filename must be a cell array of strings'); end

% only one file supported
filename = filename{1};

% read in all of the structures in the raw file
structures = readParadoppBinary(filename);

% first three sections are header, head and user configuration
hardware = structures.Id5;
head     = structures.Id4;
user     = structures.Id0;

% the rest of the sections are awac data
nsamples = length(structures.Id32);
ncells   = user.NBins;

% preallocate memory for all sample data
time         = zeros(nsamples, 1);
distance     = zeros(ncells,   1);
analn1       = zeros(nsamples, 1);
battery      = zeros(nsamples, 1);
analn2       = zeros(nsamples, 1);
heading      = zeros(nsamples, 1);
pitch        = zeros(nsamples, 1);
roll         = zeros(nsamples, 1);
pressure     = zeros(nsamples, 1);
temperature  = zeros(nsamples, 1);
velocity1    = zeros(nsamples, ncells);
velocity2    = zeros(nsamples, ncells);
velocity3    = zeros(nsamples, ncells);
backscatter1 = zeros(nsamples, ncells);
backscatter2 = zeros(nsamples, ncells);
backscatter3 = zeros(nsamples, ncells);

%
% calculate distance values from metadata. See continentalParse.m 
% inline comments for a brief discussion of this process
%
% http://www.nortek-as.com/en/knowledge-center/forum/hr-profilers/736804717
%
freq       = head.Frequency; % this is in KHz
cellStart  = user.T2;        % counts
cellLength = user.BinLength; % counts
factor     = 0;              % used in conversion

switch freq
  case 600,  factor = 0.0797;
  case 1000, factor = 0.0478;
end

cellLength = (cellLength / 256) * factor * cos(25 * pi / 180);
cellStart  =  cellStart         * 0.0229 * cos(25 * pi / 180) - cellLength;

distance(:) = (cellStart):  ...
           (cellLength): ...
           (cellStart + (ncells-1) * cellLength);
       
% Note this is actually the distance between the ADCP's transducers and the
% middle of each cell
% See http://www.nortek-bv.nl/en/knowledge-center/forum/current-profilers-and-current-meters/579860330
distance = distance + cellLength;

% retrieve sample data
time            = structures.Id32.Time';
analn1          = structures.Id32.Analn1';
battery         = structures.Id32.Battery';
analn2          = structures.Id32.Analn2';
heading         = structures.Id32.Heading';
pitch           = structures.Id32.Pitch';
roll            = structures.Id32.Roll';
pressure        = structures.Id32.PressureMSB'*65536 + structures.Id32.PressureLSW';
temperature     = structures.Id32.Temperature';
velocity1       = structures.Id32.Vel1';
velocity2       = structures.Id32.Vel2';
velocity3       = structures.Id32.Vel3';
backscatter1    = structures.Id32.Amp1';
backscatter2    = structures.Id32.Amp2';
backscatter3    = structures.Id32.Amp3';
clear structures;

% battery     / 10.0   (0.1 V    -> V)
% heading     / 10.0   (0.1 deg  -> deg)
% pitch       / 10.0   (0.1 deg  -> deg)
% roll        / 10.0   (0.1 deg  -> deg)
% pressure    / 1000.0 (mm       -> m)   assuming equivalence to dbar
% temperature / 100.0  (0.01 deg -> deg)
% velocities  / 1000.0 (mm/s     -> m/s) assuming earth coordinates
% backscatter * 0.45   (counts   -> dB)  see http://www.nortek-as.com/lib/technical-notes/seditments
battery      = battery      / 10.0;
heading      = heading      / 10.0;
pitch        = pitch        / 10.0;
roll         = roll         / 10.0;
pressure     = pressure     / 1000.0;
temperature  = temperature  / 100.0;
velocity1    = velocity1    / 1000.0;
velocity2    = velocity2    / 1000.0;
velocity3    = velocity3    / 1000.0;
backscatter1 = backscatter1 * 0.45;
backscatter2 = backscatter2 * 0.45;
backscatter3 = backscatter3 * 0.45;

sample_data = struct;
    
sample_data.toolbox_input_file              = filename;
sample_data.meta.featureType                = 'timeSeriesProfile';
sample_data.meta.head                       = head;
sample_data.meta.hardware                   = hardware;
sample_data.meta.user                       = user;
sample_data.meta.binSize                    = cellLength;
sample_data.meta.instrument_make            = 'Nortek';
sample_data.meta.instrument_model           = 'AWAC';
sample_data.meta.instrument_serial_no       = hardware.SerialNo;
sample_data.meta.instrument_sample_interval = median(diff(time*24*3600));
sample_data.meta.instrument_firmware        = hardware.FWversion;
sample_data.meta.beam_angle                 = 25;   % http://www.hydro-international.com/files/productsurvey_v_pdfdocument_19.pdf

% add dimensions with their data mapped
dims = {
    'TIME',             time,       ''; ...
    'DIST_ALONG_BEAMS', distance,   'Nortek instrument data is not vertically bin-mapped (no tilt correction applied). Cells are lying parallel to the beams, at heights above sensor that vary with tilt.'
    };
clear time distance;

nDims = size(dims, 1);
sample_data.dimensions = cell(nDims, 1);
for i=1:nDims
    sample_data.dimensions{i}.name         = dims{i, 1};
    sample_data.dimensions{i}.typeCastFunc = str2func(netcdf3ToMatlabType(imosParameters(dims{i, 1}, 'type')));
    sample_data.dimensions{i}.data         = sample_data.dimensions{i}.typeCastFunc(dims{i, 2});
    sample_data.dimensions{i}.comment      = dims{i, 3};
end
clear dims;

% add variables with their dimensions and data mapped.
% we assume no correction for magnetic declination has been applied
vars = {
    'TIMESERIES',       [],    1; ...
    'LATITUDE',         [],    NaN; ...
    'LONGITUDE',        [],    NaN; ...
    'NOMINAL_DEPTH',    [],    NaN; ...
    'VCUR_MAG',         [1 2], velocity2; ... % V
    'UCUR_MAG',         [1 2], velocity1; ... % U
    'WCUR',             [1 2], velocity3; ...
    'ABSI1',            [1 2], backscatter1; ...
    'ABSI2',            [1 2], backscatter2; ...
    'ABSI3',            [1 2], backscatter3; ...
    'TEMP',             1,     temperature; ...
    'PRES_REL',         1,     pressure; ...
    'VOLT',             1,     battery; ...
    'PITCH',            1,     pitch; ...
    'ROLL',             1,     roll; ...
    'HEADING_MAG',      1,     heading
    };
clear analn1 analn2 time distance velocity1 velocity2 velocity3 ...
    backscatter1 backscatter2 backscatter3 ...
    temperature pressure battery pitch roll heading;

nVars = size(vars, 1);
sample_data.variables = cell(nVars, 1);
for i=1:nVars
    sample_data.variables{i}.name         = vars{i, 1};
    sample_data.variables{i}.typeCastFunc = str2func(netcdf3ToMatlabType(imosParameters(vars{i, 1}, 'type')));
    sample_data.variables{i}.dimensions   = vars{i, 2};
    if ~isempty(vars{i, 2}) % we don't want this for scalar variables
        if length(sample_data.variables{i}.dimensions) == 2
            sample_data.variables{i}.coordinates = 'TIME LATITUDE LONGITUDE DIST_ALONG_BEAMS';
        else
            sample_data.variables{i}.coordinates = 'TIME LATITUDE LONGITUDE NOMINAL_DEPTH';
        end
    end
    sample_data.variables{i}.data         = sample_data.variables{i}.typeCastFunc(vars{i, 3});
end
clear vars;

%
% if wave data files are present, read them in
%
waveData = readAWACWaveAscii(filename);

% no wave data, no problem
if isempty(waveData), return; end

% turn sample data into a cell array
temp{1} = sample_data;
sample_data = temp;
clear temp;

% copy wave data into a sample_data struct; start with a copy of the 
% first sample_data struct, as all the metadata is the same
sample_data{2} = sample_data{1};

[filePath, fileRadName, ~] = fileparts(filename);
filename = fullfile(filePath, [fileRadName '.wap']);

sample_data{2}.toolbox_input_file              = filename;
sample_data{2}.meta.head                       = [];
sample_data{2}.meta.hardware                   = [];
sample_data{2}.meta.user                       = [];
sample_data{2}.meta.instrument_sample_interval = median(diff(waveData.Time*24*3600));

% we assume no correction for magnetic declination has been applied

% add dimensions with their data mapped
dims = {
    'TIME',                   waveData.Time; ...
    'FREQUENCY_1',            waveData.pwrFrequency; ...
    'FREQUENCY_2',            waveData.dirFrequency; ...
    'DIR_MAG',                waveData.Direction
    };

nDims = size(dims, 1);
sample_data{2}.dimensions = cell(nDims, 1);
for i=1:nDims
    sample_data{2}.dimensions{i}.name         = dims{i, 1};
    sample_data{2}.dimensions{i}.typeCastFunc = str2func(netcdf3ToMatlabType(imosParameters(dims{i, 1}, 'type')));
    sample_data{2}.dimensions{i}.data         = sample_data{2}.dimensions{i}.typeCastFunc(dims{i, 2});
end
clear dims;

% add variables with their dimensions and data mapped
vars = {
    'TIMESERIES',   [],      1; ...
    'LATITUDE',     [],      NaN; ...
    'LONGITUDE',    [],      NaN; ...
    'NOMINAL_DEPTH',[],      NaN; ...
    'VDEN',         [1 2],   waveData.pwrSpectrum; ... % sea_surface_wave_variance_spectral_density
    'SSWD_MAG',     [1 3],   waveData.dirSpectrum; ... % sea_surface_wave_direction_spectral_density
    'VAVH',         1,       waveData.SignificantHeight; ... % sea_surface_wave_significant_height
    'VAVT',         1,       waveData.MeanZeroCrossingPeriod; ... % sea_surface_wave_zero_upcrossing_period
    'VDIR_MAG',     1,       waveData.MeanDirection; ... % sea_surface_wave_from_direction
    'SSDS_MAG',     1,       waveData.DirectionalSpread; ... % sea_surface_wave_directional_spread
    'TEMP',         1,       waveData.Temperature; ...
    'PRES_REL',     1,       waveData.MeanPressure; ...
    'VOLT',         1,       waveData.Battery; ...
    'HEADING_MAG',  1,       waveData.Heading; ...
    'PITCH',        1,       waveData.Pitch; ...
    'ROLL',         1,       waveData.Roll; ...
    'SSWV_MAG',     [1 3 4], waveData.fullSpectrum; ... % sea_surface_wave_magnetic_directional_variance_spectral_density
    'SPCT',         1,       waveData.SpectraType % awac_spectra_calculation_method
    };
clear waveData;

nVars = size(vars, 1);
sample_data{2}.variables = cell(nVars, 1);
for i=1:nVars
    sample_data{2}.variables{i}.name         = vars{i, 1};
    sample_data{2}.variables{i}.typeCastFunc = str2func(netcdf3ToMatlabType(imosParameters(vars{i, 1}, 'type')));
    sample_data{2}.variables{i}.dimensions   = vars{i, 2};
    if ~isempty(vars{i, 2}) && ~strcmpi(vars{i, 1}, 'SPCT') % we don't want this for scalar variables nor SPCT
        if any(strcmpi(vars{i, 1}, {'VDEN', 'SSWD_MAG', 'VAVH', 'VAVT', 'VDIR_MAG', 'SSDS_MAG', 'SSWV_MAG'}))
            sample_data{2}.variables{i}.coordinates = 'TIME LATITUDE LONGITUDE'; % data at the surface, can be inferred from standard/long names
        else
            sample_data{2}.variables{i}.coordinates = 'TIME LATITUDE LONGITUDE NOMINAL_DEPTH';
        end
    end
    sample_data{2}.variables{i}.data         = sample_data{2}.variables{i}.typeCastFunc(vars{i, 3});
end
clear vars;