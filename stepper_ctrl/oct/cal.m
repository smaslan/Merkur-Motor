%----------------------------------------------------------------------------------------------------
% 
%                    
% (c) Stanislav Maslan, 2024, s.maslan@seznam.cz
% The script is distributed under MIT license, https://opensource.org/licenses/MIT.
%----------------------------------------------------------------------------------------------------
clc;
clear all;
%close all;
warning('off');

% currently best one, works only in -gui.exe!
graphics_toolkit('gnuplot');
%graphics_toolkit('qt');

% current path
rpth = fileparts(mfilename('fullpath'));
cd(rpth);

% RS232 communication package
pkg load instrument-control;


% inital COM port setup
port = 'COM9';
baud = 9600;

% timeout [in 100ms multiples]
timeout = 10;

% open port
com = serialport(port, baud);
set(com, 'Timeout', timeout);


speed_list = linspace(0,200,10);

min_amp = 100;
max_amp = 255;
min_spd = 50;
max_spd = 150;

for k = 1:numel(speed_list)

    spd = speed_list(k)    
    amp = min(max((spd - min_spd)/(max_spd - min_spd)*(max_amp - min_amp) + min_amp, min_amp),max_amp)    
        
    fprintf(com, 'SPEED %.0f;AMP %.0f\n', spd, amp);
    
    pause(1.0)

endfor


% close port and pray for success
clear com;




