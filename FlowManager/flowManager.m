function flowManager()
%FLOWMANAGER Manages the overall flow of IMOS toolbox execution.
%
% Author: Paul McCarthy <paul.mccarthy@csiro.au>
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

  % import data
  [fieldTrip sample_data skipped] = importManager();

  % display data
  displayManager(fieldTrip, sample_data,...
                @autoQCRequestCallback,...
                @manualQCRequestCallback,...
                @exportRequestCallback);
end

function sample_data = ...
  autoQCRequestCallback(sample_data, updateCallback)
%AUTOQCREQUESTCALLBACK Called when the user chooses to run auto QC
% routines over the data. Delegates to the autoQCManager.
%

  sample_data = autoQCManager(sample_data);
  
  for k = 1:length(sample_data), updateCallback(sample_data{k}); end
end

function manualQCRequestCallback()
  disp('manualQCRequestCallback');

  %qcd_data = manualQCManager(selected_qc_routine data);
  %display(qcd_data);

end

function exportRequestCallback()
  disp('exportRequestCallback');

  %for d in data
  %  exportNetCDF(d);
end
  