function ensembles = readWorkhorseEnsembles_vmdas( filename )
%READWORKHORSEENSEMBLES Reads in and returns all of the ensembles contained
% in the given binary file retrieved from a Workhorse ADCP.
%
% This function parses a binary file retrieved from a Teledyne RD Workhorse
% ADCP instrument. This function is only able to interpret raw files in the
% PD0 format (as it stands at December 2008). See  WorkHorse Commands and
% Ouput Data Format Document (P/N 957-6156-00)
%
%
% A raw Workhorse data file consists of a set of 'ensembles'. Each ensemble
% contains data for one sample period. An ensemble is made up of a number
% of sections, the last five of which may or may not be present:
%   - Header:                Ensemble information (size/contents). Always
%                            present
%   - Fixed Leader Data:     ADCP configuration, serial number etc. Always
%                            present
%   - Variable Leader Data:  Time, temperature, salinity etc. Always
%                            present.
%   - Velocity:              Current velocities for each depth (a.k.a
%                            'bins' or 'cells').  Technically optional but
%                            we are wasting our time if it's not here!
%   - Correlation Magnitude: 'Magnitude of the normalized echo
%                            autocorrelation at the lag used for estimating
%                            the Doppler phase change'.
%   - Echo Intensity:        Echo intensity data.
%   - Percent Good:          Percentage of good data for each depth cell.
%   - Bottom Track Data:     Bottom track data.  (not really necessary for
%                            bottom moored upward looking ADCP)
%
% This function parses the ensembles, and returns all of them in a cell
% array.
%
% Inputs:
%   filename  - Raw binary data file retrieved from a Workhorse.
%
% Outputs:
%   ensembles - Scalar structure of ensembles.
%
% Author:       Paul McCarthy <paul.mccarthy@csiro.au>
% Contributor:  Charles James <charles.james@sa.gov.au>
%               Guillaume Galibert <guillaume.galibert@utas.edu.au>

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

% ensure that there is exactly one argument,
% and that it is a string
error(nargchk(1, 1, nargin));
if ~ischar(filename), error('filename must be a string');
end

% check that file exists
if isempty(dir(filename)), error([filename ' does not exist']); end

% read in the whole file into 'data'
fid = -1;
data = [];
try
    fid = fopen(filename, 'rb');
    data = fread(fid, inf, '*uint8');
    fclose(fid);
catch e
    if fid ~= -1, fclose(fid); end
    rethrow e;
end

% The following rules are used in creating our BroadBand Data Format of PD0.
% Our recommended decoding sequence is presented next.
%
% Rules for the BroadBand Data Format PD0:
%
%     - All data types (i.e. fixed leader, variable leader, velocity, echo
%     intensity, correlation, percent good, etc.) will be given a specific
%     and unique ID number.
%     - Once a data type has been given an ID number the format of the data
%     inside that ID number will never change in units, order, or number of
%     bytes.
%     - Data may be added to an existing data type only by adding the bytes
%       to the end of the data format. As an example, the variable leader
%       data contains information on ensemble number, time, heading, pitch,
%       roll, temperature, pressure, etc. The format for the bytes 1-53 are
%       now specified by changes added in support to the WH ADCP. If
%       additional sensor data is to be added to the variable leader data
%       then it must be added to the end of the data string (bytes 54-x as
%       an example).
%     - The order of data types in an ensemble are not fixed. That is there
%       is no guarantee that velocity data will always be output before
%       correlation data.
%     - The header data will include the number of data types in the files
%       and the offset to each ID number for each data type.
%     - The total number of the bytes in an ensemble minus the 2 byte
%       checksum will be included in the header.
%
% Recommended Data Decoding Sequence for BroadBand Data Format PD0:
%
%     -Locate the header data by locating the header ID number (in the case
%     of PD0 profile data that will be 7F7F).
%     -Confirm that you have the correct header ID by:
%
%         2a. Locating the total number of bytes (located in the header data) in the ensemble
%         2b. Add 2 bytes to the value in 2a. This will be your offset to the next ensemble.
%         2c. Read the 2 bytes following the offset to the next ensemble (calculated in step 2b).
%         2d. Confirming that the next 2 bytes are the header ID number.
%         If it is then you have located the Header ID. If not then go back to step 1 and search for the next header ID number occurrence.
%
%     -Locate the number of data types (located in the header data).
%     -Locate the offset to each data type (located in the header data).
%     -Locate the data ID type you wish to decode by using the offset to each data type and confirm the data ID number at that offset matches the ID type you are looking for.
%     -Once the proper ID type has been located use the DVL Technical Manual for the DVL you are using to understand what each byte represents in that particular data type.

% get ensemble start key
headerID        = 127;      % hexadecimal '7F'
dataSourceID    = 127;      % hexadecimal '7F'
% WH headerID=127, dataSourceID=127
% note other IDs identified
% dataSourceID=121 (hex '79') probably for waves burst samples (ignore here)

% try and parse all ensembles at once
% we will try this in a couple of steps

% ensemble is indicated by header and data source IDs in the data field;
% idx will indicate the start of a possible ensemble
idh = data(1:end-1) == headerID;
ids = data(2:end) == dataSourceID;
idx = find(idh & ids);
clear idh ids;

% this will have found far more than the actual number of ensembles as
% every pair matching the ID numbers will be interpreted as an ensemble start;

% number of bytes in ensemble (not including the 2 checksum bytes)
[~, ~, cpuEndianness] = computer;
bpe     = [data(idx(:)+2) data(idx(:)+3)]';
nBytes  = bytecast(bpe(:), 'L', 'uint16', cpuEndianness);
clear bpe;
% number of bytes in ensemble
nLen = nBytes + 2;
lenData = length(data);

% keep funky eLen values from putting us out of bounds
oob = (idx+nLen-1) > lenData;
idx(oob)    = [];
nBytes(oob) = [];
nLen(oob)   = [];
clear oob;

% check sum is in last two bytes
givenCrc = indexData(data, idx+nLen-2, idx+nLen-1, 'uint16', cpuEndianness)';
clear nLen;

% could use indexData to recover data between idx and idx+nBytes-1
% but at this point nBytes could be as big as FFFFh so ...
% to compute Crc sum over each ensemble

% !!! double(data) may be too big to be handled with current free memory
ctype = computer;

switch ctype
    case 'PCWIN'
        % Let's consider memory issues
        userview = memory;
        
        % 10% margin
        bytesAvailable = userview.MaxPossibleArrayBytes - 10*userview.MaxPossibleArrayBytes/100;
        
        % let's see if cumsum(double(data)) fit
        bytesNeeded = lenData*8 * 2;
        if bytesNeeded < bytesAvailable
            dataSum = cumsum(double(data));
        else
            % we have to chop the process in a number of times the current free memory
            % allow us to, with the hack of casting dataSum in uint32
            n = ceil((bytesNeeded) / bytesAvailable);
            dataSum = [];
            for i=1:n
                iStart = floor((i-1)*lenData/n) + 1;
                iEnd   = floor(i*lenData/n);
                if isempty(dataSum)
                    dataSum = cumsum(double(data(iStart:iEnd)));
                else
                    dataSum = [dataSum; cumsum(double(data(iStart:iEnd)))+dataSum(end)];
                end
            end
        end
        
    otherwise
        % We suppose we can read the whole file at once without any memory
        % trouble
        dataSum = cumsum(double(data));
end

% idx is the start of the sum sequence, iend is the end
iend = idx+nBytes-1;

% following formula gives total sum between idx and iend
calcCrc = double(data(idx)) + double(dataSum(iend) - dataSum(idx));
clear dataSum;

% this is the checksum formula
calcCrc = bitand(calcCrc, 65535);

% only good ensembles should pass the checksum test
good = (calcCrc == givenCrc);

% define ensemble variables
idx     = idx(good);
nBytes  = nBytes(good);

% checksum should have been sufficent
% but I have still found ensembles that don't
% have matching bytes (probably due to sheer number of ensembles there is a
% finite chance of an erroneous checksum match)
%
% Difference in index should be equal to nBytes or
% we have a problem.

% calculate difference in index-2 from start to end of data
didx = [diff(idx); lenData - idx(end) + 1] - 2;

% compare with number of Bytes to find errors
idodgy = (didx ~= nBytes);

if any(idodgy)
    % wow! we need to keep trying to fix;
    % at this point I am assuming that nBytes is the same for all good records,
    % since this information is stored in every ensemble this may eventually
    % break down for unseen formats.
    if std(nBytes(~idodgy)) == 0
        % we only have one byte length in data record
        nBytes0 = nBytes(~idodgy);
        igood = (nBytes == nBytes0(1));
        
        idx     = idx(igood);
        nBytes  = nBytes(igood);
    else
        error('this is interesting, need to look at this data file please contact developer');
    end
end

% in principle each ensemble could have a different number of data
% types
nDataTypes = double(data(idx+5));

% in each ensemble bytes 6:2*nDataTypes give offsets to data
dataOffsets = indexData(data, idx+6, idx+6+2*nDataTypes-1, 'uint16', cpuEndianness);
[i j] = size(dataOffsets);

% create a big index matrix!
IDX = meshgrid(idx, 1:i) + dataOffsets;

% get rid of possible NaN
iNaN = isnan(IDX);
IDXnoNan = IDX(~iNaN);

% what type of sections do we have in each ensemble;
sType = indexData(data, IDXnoNan(:), IDXnoNan(:)+1, 'uint16', cpuEndianness);

iFixedLeader    = IDX(sType == 0);
iVarLeader      = IDX(sType == 128);
iVel            = IDX(sType == 256);
iCorr           = IDX(sType == 512);
iEcho           = IDX(sType == 768);
iPCgood         = IDX(sType == 1024);

%iStatProf      = IDX(sType == 1280);
iBTrack         = IDX(sType == 1536);
%iMicroCat      = IDX(sType == 2048);

iVMDAS = IDX(sType == 8192); % hex 2000
iWinRiver2 = IDX(sType == 8226); % hex 2022

% major change to PM code - ensembles is a scalar structure not cell array
ensembles.fixedLeader       = parseFixedLeader(data, iFixedLeader, cpuEndianness);
ensembles.variableLeader    = parseVariableLeader(data, iVarLeader, cpuEndianness);

% subfields contain the vector time series
% we set a static value for nCells to the most frequent value found
nCells = mode(ensembles.fixedLeader.numCells);

ensembles.velocity          = parseVelocity(data, nCells, iVel, cpuEndianness);

if ~isempty(iCorr)
    ensembles.corrMag       = parseX(data, nCells, 'corrMag', iCorr, cpuEndianness);
end

if ~isempty(iEcho)
    ensembles.echoIntensity = parseX(data, nCells, 'echoIntensity', iEcho, cpuEndianness);
end

if ~isempty(iPCgood)
    ensembles.percentGood   = parseX(data, nCells, 'percentGood', iPCgood, cpuEndianness);
end

if ~isempty(iBTrack)
    ensembles.bottomTrack   = parseBottomTrack(data, iBTrack, cpuEndianness);
end

if ~isempty(iVMDAS)
    ensembles.nav   = parseVMDAS(data, iVMDAS, cpuEndianness);
end

end

function dsub = indexData(data, istart, iend, dtype, cpuEndianness)
% function dsub = indexData(data, istart, iend, dtype)
% vectorized data index algorithm, converts data between istart and iend
% from binary 8bit data to dtype, where istart and iend can be vectors!
% Charles James


% create matrix of indicies
width = max(iend-istart+1);

[I J]   = meshgrid(istart, 0:width-1);
K       = meshgrid(iend,   0:width-1);

IND = I + J;

ibad = IND > K;

if any(any(ibad))
    % We assume that if an ensemble contain one bad data, all data are bad
    % in this ensemble
    ibad(:,any(ibad)) = ones(size(ibad,1), 1);
end

datatst = data(IND(:));
datatst(ibad) = 0;
dsub = bytecast(datatst, 'L', dtype, cpuEndianness);
ibad = bytecast(uint8(ibad(:)), 'L', dtype, cpuEndianness);

i = length(istart);
j = length(dsub)/i;

dsub = reshape(dsub, j, i);
ibad = reshape(ibad, j, i) ~= 0;

dsub(ibad) = nan;

end

function [sect len] = parseFixedLeader(data, idx, cpuEndianness)
%PARSEFIXEDLEADER Parses a fixed leader section from an ADCP ensemble.
%
% Inputs:
%   data - vector of raw bytes.
%   idx  - index that the section starts at.
%
% Outputs:
%   sect - struct containing the fields that were parsed from the fixed
%          leader section.
%   len  - number of bytes that were parsed.
%

sect = struct;
len = 59;

sect.fixedLeaderId       = indexData(data, idx, idx+1, 'uint16', cpuEndianness)';
% 8,9,16 - WH navigator
% 10 -rio grande
% 15, 17 - NB
% 19 - REMUS, or customer specific
% 11- H-ADCP
% 31 - Streampro
% 34 - NEMO
% 50 - WH, no bottom track (built on 16.31)
% 51 - WH, w/ bottom track
% 52 - WH, mariner
sect.cpuFirmwareVersion  = double(data(idx+2));
sect.cpuFirmwareRevision = double(data(idx+3));
prog_ver = sect.cpuFirmwareVersion + sect.cpuFirmwareRevision/100;
% sect.systemConfiguration = indexData(data, idx+4, idx+5, 'uint16', cpuEndianness)';
sect.systemConfigurationLSB = double(data(idx+4));
sect.systemConfigurationMSB = double(data(idx+5));
LSB = dec2bin(double(data(idx+4)));
MSB = dec2bin(double(data(idx+5)));
while(size(LSB, 2) < 8), LSB = [repmat('0', size(LSB, 1), 1) LSB]; end
while(size(MSB, 2) < 8), MSB = [repmat('0', size(MSB, 1), 1) MSB]; end
sect.systemConfiguration = [LSB MSB];

sect.beam_angle=getopt(bitand(sect.systemConfigurationMSB,3),15,20,30);
sect.num_beams=getopt(bitand(sect.systemConfigurationMSB,16)==16,4,5);

sect.freq = getopt( bitand(sect.systemConfigurationLSB,7),75,150,300,600,1200,2400,38);
sect.beamPattern   =getopt(bitand(sect.systemConfigurationLSB,8)==8,'concave','convex'); % 1=convex,0=concave
sect.orientation    =getopt(bitand(sect.systemConfigurationLSB,128)==128,'down','up');    % 1=up,0=down

sect.beamConfig = getopt( bitand(sect.systemConfigurationMSB,240),'4-BEAM JANUS','5-BM JANUS CFIG DEMOD','5-BM JANUS CFIG(2 DEMD)');

sect.realSimFlag         = double(data(idx+6));
sect.lagLength           = double(data(idx+7));
sect.numBeams            = double(data(idx+8));
sect.numCells            = double(data(idx+9));
block                    = indexData(data, idx+10, idx+15, 'uint16', cpuEndianness)';
sect.pingsPerEnsemble    = block(:,1);
sect.depthCellLength     = block(:,2);
sect.blankAfterTransmit  = block(:,3);
sect.profilingMode       = double(data(idx+16));
sect.lowCorrThresh       = double(data(idx+17));
sect.numCodeReps         = double(data(idx+18));
sect.gdMinimum           = double(data(idx+19));
sect.errVelocityMax      = indexData(data, idx+20, idx+21, 'uint16', cpuEndianness)';
sect.tppMinutes          = double(data(idx+22));
sect.tppSeconds          = double(data(idx+23));
sect.tppHundredths       = double(data(idx+24));
sect.coordinateTransform = double(data(idx+25));

%sect.coord=dec2base(coord_sys,2,8);
sect.coordSys      =getopt(bitand(bitshift(sect.coordinateTransform,-3),3),'beam','instrument','ship','earth');
sect.usePitchroll  =getopt(bitand(sect.coordinateTransform,4)==4,false,true);
sect.use3beam      =getopt(bitand(sect.coordinateTransform,2)==2,false,true);
sect.binMapping    =getopt(bitand(sect.coordinateTransform,1)==1,false,true);

block                    = indexData(data, idx+26, idx+29, 'int16', cpuEndianness)';
sect.headingAlignment    = block(:,1);
sect.headingBias         = block(:,2);
sect.sensorSource        = dec2bin(double(data(idx+30)));
while(size(sect.sensorSource, 2) < 8), sect.sensorSource = [repmat('0', size(sect.sensorSource, 1), 1) sect.sensorSource]; end
sect.sensorsAvailable    = dec2bin(double(data(idx+31)));
while(size(sect.sensorsAvailable, 2) < 8), sect.sensorsAvailable = [repmat('0', size(sect.sensorsAvailable, 1), 1) sect.sensorsAvailable]; end
block                    = indexData(data, idx+32, idx+35, 'uint16', cpuEndianness)';
sect.bin1Distance        = block(:,1);
sect.xmitPulseLength     = block(:,2);
sect.wpRefLayerAvgStart  = double(data(idx+36));
sect.wpRefLayerAvgEnd    = double(data(idx+37));
sect.falseTargetThresh   = double(data(idx+38));
% byte 40 is spare

sect.transmitLagDistance = indexData(data, idx+40, idx+41, 'uint16', cpuEndianness)';

%don't understand cpuBoardSerialNo calculation because
%dec2hex(cpuBoardSerialNo) != cpuBoardSerialNoHEX
sect.cpuBoardSerialNo    = indexData(data, idx+42, idx+49, 'uint64', cpuEndianness)';
%sect.cpuBoardSerialNo    = uint64(bytecast(data(idx+42:idx+49), 'L', 'uint64'));
sect.cpuBoardSerialNoHEX=sprintf('%s',dec2hex(data(idx+42:idx+49))');

%sect.systemBandwidth     = indexData(data, idx+50, idx+51, 'uint16')';
%sect.systemPower         = double(data(idx+52));
% % byte 54 is spare
% % following two fields are not used for Firmware before 16.30
% if (sect.cpuFirmwareVersion >= 16) && (sect.cpuFirmwareRevision >= 30)
%     sect.instSerialNumber    = num2str(uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32')));
%     %sect.instSerialNumber    = uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32'));
%     sect.beamAngle           = double(data(idx+58));
% else
%     sect.instSerialNumber    = '';
%     sect.beamAngle           = NaN;
% end

% just in case we have some old data laying around
sect.systemBandwidth=NaN;
sect.systemPower=NaN;
sect.beamAngle=NaN;
% section of code cobbled together from rdradcp.m and rdpadcp.m by R. Pawlowicz (rich@ocgy.ubc.ca)
if sect.cpuFirmwareVersion==8 | sect.cpuFirmwareVersion==10 | sect.cpuFirmwareVersion==16 ...
        | sect.cpuFirmwareVersion==50 | sect.cpuFirmwareVersion==51 | sect.cpuFirmwareVersion==52
    
    if prog_ver>=8.14  % Added CPU serial number with v8.14
        sect.instSerialNumber = num2str(uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32', cpuEndianness)));
    end
    
    if prog_ver>=8.24  % Added 2 more bytes with v8.24 firmware
        sect.systemBandwidth = indexData(data, idx+50, idx+51, 'uint16', cpuEndianness)';
    end
    
    if prog_ver>=16.05  % Added 1 more bytes with v16.05 firmware
        sect.systemPower = double(data(idx+52));
        %nbyte=nbyte+1;
    end
    
    if prog_ver>=16.27  % Added bytes for REMUS, navigators, and HADCP
        % not sure where these occur, what is the size of fixed leader on
        % these types?
        sect.navigator_basefreqindex=NaN;%fread(fd,1,'uint8'); %54
        %nbyte=nbyte+1;
        sect.remus_serialnum=NaN;%fread(fd,4,'uint8'); %55
        %nbyte=nbyte+4;
        sect.h_adcp_beam_angle=NaN;%fread(fd,1,'uint8'); %59
        %nbyte=nbyte+1;
    end
    
    if prog_ver>=16.30
        sect.instSerialNumber    = num2str(uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32', cpuEndianness)));
        %sect.instSerialNumber    = uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32'));
        sect.beamAngle           = double(data(idx+58));
    end
    
elseif sect.cpuFirmwareVersion==9,
    
    if prog_ver>=9.10,  % Added CPU serial number with v8.14
        sect.instSerialNumber = num2str(uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32', cpuEndianness)));
        %nbyte=nbyte+8;
        sect.systemBandwidth = indexData(data, idx+50, idx+51, 'uint16', cpuEndianness)';
        %nbyte=nbyte+2;
    end
    
elseif sect.cpuFirmwareVersion==14 | sect.cpuFirmwareVersion==23,
    sect.instSerialNumber = num2str(uint32(bytecast(data(idx+54:idx+57), 'L', 'uint32', cpuEndianness)));
else
      nEnsemble = length(idx);
      sect.instSerialNumber    = NaN(nEnsemble, 1);
      sect.beamAngle           = NaN(nEnsemble, 1);
  
end

end

function [sect len] = parseVariableLeader( data, idx, cpuEndianness)
%PARSEVARIABLELEADER Parses a variable leader section from an ADCP ensemble.
%
% Inputs:
%   data - vector of raw bytes.
%   idx  - index that the section starts at.
%
% Outputs:
%   sect - struct containing the fields that were parsed from the
%          variable leader section.
%   len  - number of bytes that were parsed.
%
sect = struct;
len = 65;

block                       = indexData(data,idx,idx+3, 'uint16', cpuEndianness)';
sect.variableLeaderId       = block(:,1);
sect.ensembleNumber         = block(:,2);
sect.rtcYear                = double(data(idx+4));
sect.rtcMonth               = double(data(idx+5));
sect.rtcDay                 = double(data(idx+6));
sect.rtcHour                = double(data(idx+7));
sect.rtcMinute              = double(data(idx+8));
sect.rtcSecond              = double(data(idx+9));
sect.rtcHundredths          = double(data(idx+10));
sect.ensembleMsb            = double(data(idx+11));
block                       = indexData(data,idx+12,idx+19, 'uint16', cpuEndianness)';
sect.bitResult              = block(:,1);
sect.speedOfSound           = block(:,2);
sect.depthOfTransducer      = block(:,3);
sect.heading                = block(:,4);
block                       = indexData(data,idx+20,idx+23, 'int16', cpuEndianness)';
sect.pitch                  = block(:,1);
sect.roll                   = block(:,2);
sect.salinity               = indexData(data,idx+24,idx+25, 'uint16', cpuEndianness)';
sect.temperature            = indexData(data,idx+26,idx+27, 'int16', cpuEndianness)';
sect.mptMinutes             = double(data(idx+28));
sect.mptSeconds             = double(data(idx+29));
sect.mptHundredths          = double(data(idx+30));
sect.hdgStdDev              = double(data(idx+31));
sect.pitchStdDev            = double(data(idx+32));
sect.rollStdDev             = double(data(idx+33));
sect.adcChannel0            = double(data(idx+34));
sect.adcChannel1            = double(data(idx+35));
sect.adcChannel2            = double(data(idx+36));
sect.adcChannel3            = double(data(idx+37));
sect.adcChannel4            = double(data(idx+38));
sect.adcChannel5            = double(data(idx+39));
sect.adcChannel6            = double(data(idx+40));
sect.adcChannel7            = double(data(idx+41));
sect.errorStatusWord        = indexData(data,idx+42,idx+45, 'uint32', cpuEndianness)';
% bytes 47-48 are spare
% note pressure is technically supposed to be an unsigned integer so when
% at surface negative values appear huge ~4 million dbar we'll read it in
% as signed integer to avoid this but need to be careful if deploying
% ADCP near centre of earth!
sect.pressure               = indexData(data,idx+48,idx+51, 'int32', cpuEndianness)';
sect.pressureSensorVariance = indexData(data,idx+52,idx+55, 'int32', cpuEndianness)';
% byte 57 is spare
sect.y2kCentury             = double(data(idx+57));
sect.y2kYear                = double(data(idx+58));
sect.y2kMonth               = double(data(idx+59));
sect.y2kDay                 = double(data(idx+60));
sect.y2kHour                = double(data(idx+61));
sect.y2kMinute              = double(data(idx+62));
sect.y2kSecond              = double(data(idx+63));
sect.y2kHundredth           = double(data(idx+64));
end

function [sect len] = parseVelocity( data, numCells, idx, cpuEndianness)
%PARSEVELOCITY Parses a velocity section from an ADCP ensemble.
%
% Inputs:
%   data     - vector of raw bytes.
%   numCells - number of depth cells/bins.
%   idx      - index that the section starts at.
%
% Outputs:
%   sect     - struct containing the fields that were parsed from the
%              velocity section.
%   len      - number of bytes that were parsed.
%
sect = struct;
nBeams = 4;
len = 2 + numCells * nBeams * 2; % 2 bytes per beam

sect.velocityId = indexData(data,idx,idx+1, 'uint16', cpuEndianness)';

idx = idx + 2;

vels = indexData(data, idx, idx + 2*nBeams*numCells'-1, 'int16', cpuEndianness)';
s = size(vels);

ibeam = 1:nBeams:s(2);

sect.velocity1 = vels(:, ibeam);
sect.velocity2 = vels(:, ibeam+1);
sect.velocity3 = vels(:, ibeam+2);
sect.velocity4 = vels(:, ibeam+3);

end

function [sect len] = parseX( data, numCells, name, idx, cpuEndianness)
%PARSEX Parses one of the correlation magnitude, echo intensity or percent
% good sections from an ADCP ensemble. They all have the same format.
%
% Inputs:
%   data     - vector of raw bytes.
%   numCells - number of depth cells/bins.
%   name     - what section is being parsed, e.g. 'correlationMagnitude',
%              'echoIntensity' or 'percentGood'. This is used as a prefix
%              for the ID field.
%   idx      - index that the section starts at.
%
% Outputs:
%   sect     - struct containing the fields that were parsed from the given
%              section.
%   len      - number of bytes that were parsed.
%
sect = struct;
nBeams = 4;
len = 2 + numCells * nBeams;

sect.([name 'Id']) = indexData(data,idx,idx+1,'uint16', cpuEndianness);

idx = idx + 2;

fields = indexData(data,idx,idx + nBeams*numCells'-1, 'uint8', cpuEndianness)';
s = size(fields);
ibeam = 1:nBeams:s(2);

sect.field1 = fields(:, ibeam);
sect.field2 = fields(:, ibeam+1);
sect.field3 = fields(:, ibeam+2);
sect.field4 = fields(:, ibeam+3);

end

function [sect length] = parseBottomTrack( data, idx, cpuEndianness)
%PARSEBOTTOMTRACK Parses a bottom track data section from an ADCP
% ensemble.
%
% Inputs:
%   data   - vector of raw bytes.
%   idx    - index that the section starts at.
%
% Outputs:
%   sect   - struct containing the fields that were parsed from trawhe bottom
%            track section.
%   length - number of bytes that were parsed.
%
sect = struct;
length = 85;

block                       = indexData(data, idx, idx+5, 'uint16', cpuEndianness)';
sect.bottomTrackId          = block(:,1);
sect.btPingsPerEnsemble     = block(:,2);
sect.btDelayBeforeReacquire = block(:,3);
sect.btCorrMagMin           = double(data(idx+6));
sect.btEvalAmpMin           = double(data(idx+7));
sect.btPercentGoodMin       = double(data(idx+8));
sect.btMode                 = double(data(idx+9));
sect.btErrVelMax            = indexData(data, idx+10, idx+11, 'uint16', cpuEndianness)';
% bytes 13-16 are spare
block                       = indexData(data, idx+16, idx+31, 'uint16', cpuEndianness)';
sect.btBeam1Range           = block(:,1);
sect.btBeam2Range           = block(:,2);
sect.btBeam3Range           = block(:,3);
sect.btBeam4Range           = block(:,4);
sect.btBeam1Vel             = block(:,5);
sect.btBeam2Vel             = block(:,6);
sect.btBeam3Vel             = block(:,7);
sect.btBeam4Vel             = block(:,8);
sect.btBeam1Corr            = double(data(idx+32));
sect.btBeam2Corr            = double(data(idx+33));
sect.btBeam3Corr            = double(data(idx+34));
sect.btBeam4Corr            = double(data(idx+35));
sect.btBeam1EvalAmp         = double(data(idx+36));
sect.btBeam2EvalAmp         = double(data(idx+37));
sect.btBeam3EvalAmp         = double(data(idx+38));
sect.btBeam4EvalAmp         = double(data(idx+39));
sect.btBeam1PercentGood     = double(data(idx+40));
sect.btBeam2PercentGood     = double(data(idx+41));
sect.btBeam3PercentGood     = double(data(idx+42));
sect.btBeam4PercentGood     = double(data(idx+43));
block                       = indexData(data, idx+44, idx+49, 'uint16', cpuEndianness)';
sect.btRefLayerMin          = block(:,1);
sect.btRefLayerNear         = block(:,2);
sect.btRefLayerFar          = block(:,3);
block                       = indexData(data, idx+50, idx+57, 'int16', cpuEndianness)';
sect.btBeam1RefLayerVel     = block(:,1);
sect.btBeam2RefLayerVel     = block(:,2);
sect.btBeam3RefLayerVel     = block(:,3);
sect.btBeam4RefLayerVel     = block(:,4);
sect.btBeam1RefCorr         = double(data(idx+58));
sect.btBeam2RefCorr         = double(data(idx+59));
sect.btBeam3RefCorr         = double(data(idx+60));
sect.btBeam4RefCorr         = double(data(idx+61));
sect.btBeam1RefInt          = double(data(idx+62));
sect.btBeam2RefInt          = double(data(idx+63));
sect.btBeam3RefInt          = double(data(idx+64));
sect.btBeam4RefInt          = double(data(idx+65));
sect.btBeam1RefGood         = double(data(idx+66));
sect.btBeam2RefGood         = double(data(idx+67));
sect.btBeam3RefGood         = double(data(idx+68));
sect.btBeam4RefGood         = double(data(idx+69));
sect.btMaxDepth             = indexData(data, idx+70, idx+71, 'uint16', cpuEndianness)';
sect.btBeam1RssiAmp         = double(data(idx+72));
sect.btBeam2RssiAmp         = double(data(idx+73));
sect.btBeam3RssiAmp         = double(data(idx+74));
sect.btBeam4RssiAmp         = double(data(idx+75));
sect.btGain                 = double(data(idx+76));
sect.btBeam1RangeMsb        = double(data(idx+77));
sect.btBeam2RangeMsb        = double(data(idx+78));
sect.btBeam3RangeMsb        = double(data(idx+79));
sect.btBeam4RangeMsb        = double(data(idx+80));
%bytes 82-85 are spare
end

function [sect length] = parseVMDAS( data, idx, cpuEndianness)
%PARSEVMDAS Parses a VMDAS nav data section from an ADCP ensemble.
%
% Inputs:
%   data   - vector of raw bytes.
%   idx    - index that the section starts at.
%
% Outputs:
%   sect   - struct containing the fields that were parsed from trawhe bottom
%            track section.
%   length - number of bytes that were parsed.
%

% Bytes 79 through 92 were added in VmDas version 1.43. How do I test for
% that?

sect = struct;
length = 92;

sect.navLeaderId       = indexData(data, idx, idx+1, 'uint16', cpuEndianness)';

sect.utcDay_s       = double(data(idx+2)); % UTC Day
sect.utcMonth_s     = double(data(idx+3)); % UTC Month
sect.utcYear_s      = indexData(data, idx+4, idx+5, 'uint16', cpuEndianness)'; % UTC Year
sect.utcSeconds_s =  indexData(data, idx+6, idx+9, 'uint32', cpuEndianness)'; % UTC time of first fix, seconds since midnight

sect.clockOffset =  indexData(data, idx+10, idx+13, 'int32', cpuEndianness)'; % UTC time of first fix, seconds since midnight

cfac=180/2^31;
% first latitude position received after the previous ADCP ping
sect.slatitude = indexData(data, idx+14, idx+17, 'int32', cpuEndianness)' * cfac;
% This is the first longitude position received after the previous ADCP ping.
sect.slongitude = indexData(data, idx+18, idx+21, 'int32', cpuEndianness)' * cfac;

% UTC Time of last fix. Time since midnight UTC; LSB = 1E-4 seconds
sect.y2kSeconds_e =  indexData(data, idx+22, idx+25, 'uint32', cpuEndianness)';
% Last Latitude. This is the last latitude position received prior to the
% current ADCP ping
sect.elatitude = indexData(data, idx+26, idx+29, 'int32', cpuEndianness)' * cfac;
% Last Longitude. This is the last longitude position received prior to the
% current ADCP ping
sect.elongitude = indexData(data, idx+30, idx+33, 'int32', cpuEndianness)' * cfac;

sect.avgSpeed = indexData(data, idx+34, idx+35, 'int16', cpuEndianness)';
sect.avgTrackTrue = indexData(data, idx+36, idx+37, 'int16', cpuEndianness)' * cfac;
sect.avgTrackMagnetic = indexData(data, idx+38, idx+39, 'int16', cpuEndianness)' * cfac;
sect.speedMadeGood = indexData(data, idx+40, idx+41, 'int16', cpuEndianness)';
sect.directionMadeGood = indexData(data, idx+42, idx+43, 'int16', cpuEndianness)' * cfac;

% bytes 45:46 (idx+44:idx+45) reserved for TRDI use.

sect.flags = indexData(data, idx+46, idx+47, 'int16', cpuEndianness)';

% bytes 49:50 (idx+48:idx+49) reserved for TRDI use.

sect.ensembleNumber = indexData(data, idx+50, idx+53, 'uint32', cpuEndianness)';
sect.ensembleYear      = indexData(data, idx+54, idx+55, 'uint16', cpuEndianness)';
sect.ensembleDay     = double(data(idx+56));
sect.ensembleMonth     = double(data(idx+57));
sect.ensembleSeconds =  indexData(data, idx+58, idx+61, 'uint32', cpuEndianness)';

sect.pitch = indexData(data, idx+62, idx+63, 'int16', cpuEndianness)' * cfac;
sect.roll = indexData(data, idx+64, idx+65, 'int16', cpuEndianness)' * cfac;
sect.heading = indexData(data, idx+66, idx+67, 'int16', cpuEndianness)' * cfac;

sect.nSpeedSamplesAveraged = indexData(data, idx+68, idx+69, 'uint16', cpuEndianness)';
sect.nTrueTrackSamplesAveraged = indexData(data, idx+70, idx+71, 'uint16', cpuEndianness)';
sect.nMagneticTrackSamplesAveraged = indexData(data, idx+72, idx+73, 'uint16', cpuEndianness)';
sect.nHeadingSamplesAveraged = indexData(data, idx+74, idx+75, 'uint16', cpuEndianness)';
sect.nPRSamplesAveraged = indexData(data, idx+76, idx+77, 'uint16', cpuEndianness)';

sect.avgTrueVelNorth = indexData(data, idx+78, idx+79, 'int16', cpuEndianness)';
sect.avgTrueVelEast = indexData(data, idx+80, idx+81, 'int16', cpuEndianness)';
sect.avgMagVelNorth = indexData(data, idx+82, idx+83, 'int16', cpuEndianness)';
sect.avgMagVelEast = indexData(data, idx+84, idx+85, 'int16', cpuEndianness)';
sect.speedMadeGoodNorth = indexData(data, idx+86, idx+87, 'int16', cpuEndianness)';
sect.speedMadeGoodEast = indexData(data, idx+88, idx+89, 'int16', cpuEndianness)';

sect.primaryFlags = indexData(data, idx+90, idx+91, 'int16', cpuEndianness)';

end

%-------------------------------------
function opt=getopt(val,varargin);
% Returns one of a list (0=first in varargin, etc.)
if val+1>length(varargin),
    opt='unknown';
else
    opt=varargin{val+1};
end
end