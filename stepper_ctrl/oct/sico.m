%----------------------------------------------------------------------------------------------------
% 
%                    
% (c) Stanislav Maslan, 2024, s.maslan@seznam.cz
% The script is distributed under MIT license, https://opensource.org/licenses/MIT.
%----------------------------------------------------------------------------------------------------
clc;
clear all;
close all;
warning('off');

% currently best one, works only in -gui.exe!
graphics_toolkit('gnuplot');
%graphics_toolkit('qt');

% current path
rpth = fileparts(mfilename('fullpath'));
cd(rpth);

phi = linspace(0,2*pi,257)(1:end-1);

data = [round(127*sin(phi));round(127*cos(phi))](:);

fw = fopen(fullfile(rpth,'sico.c'),'wt');
fprintf(fw, 'const int8_t sico[] PROGMEM = {\n    ');
for k = 1:numel(data)
    fprintf(fw, '%4d', data(k));
    if k == numel(data)
        fprintf(fw, '};\n');
    elseif mod(k,16) == 0
        fprintf(fw, ',\n    ');
    else
        fprintf(fw, ',');
    endif        
endfor

fclose(fw);
