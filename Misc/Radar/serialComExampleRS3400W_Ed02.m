
%% Establish Serial communication
% Open and configure serial port
% serialOne = serial('COM1','BaudRate',115200,'DataBits',8);
% set(serialOne, 'OutputBufferSize',4096);
% set(serialOne, 'InputBufferSize',4096);

serialOne = serial('COM5','BaudRate',9600,'DataBits',8);
set(serialOne, 'OutputBufferSize',4096);
set(serialOne, 'InputBufferSize',4096);

set(serialOne, 'Terminator', 'CR');
fopen(serialOne);
pause(0.2);
% Clear buffer
while(serialOne.BytesAvailable >0)
    fscanf(serialOne);
end

%% Configure Radar
% Set FMCW parameter values
centerFreq = 76.5e9;
freqSpan = 1.0e9;
freqPoints = 1024;
sweepNumbers = 1;
sweepTime = 0.075;

% Send configuration commands
fprintf(serialOne,'HARD:SYST W');
fscanf(serialOne)
fprintf(serialOne,'INIT');
fscanf(serialOne)
fprintf(serialOne,'SWEEP:MEAS ON');
fscanf(serialOne)
fprintf(serialOne,'TRIG:SOURCE:IMMEDIATE');
fscanf(serialOne)
fprintf(serialOne,['FREQ:CENTER ' num2str(centerFreq)]);
fscanf(serialOne)
fprintf(serialOne,['FREQ:SPAN ' num2str(freqSpan)]);
fscanf(serialOne)
fprintf(serialOne,['FREQ:POINTS ' num2str(freqPoints)]);
fscanf(serialOne)
fprintf(serialOne,['SWEEP:NUMBERS ' num2str(sweepNumbers)]);
fscanf(serialOne)
fprintf(serialOne,['SWEEP:TIME ' num2str(sweepTime)]);
fscanf(serialOne)

%% Peform Sweep and plot IF signal and FFT
% Sweep and request trace
fprintf(serialOne,'TRIG:ARM');
fscanf(serialOne)
fprintf(serialOne,'TRACE:RAW ?');
% Read trace
trace_uint8 = uint8(fread(serialOne, freqPoints * 2, 'uint8'));
trace_uint8

% Typecast from uint8(one byte) to uint16 (two bytes)
trace = double(typecast(trace_uint8, 'uint16'));

% Plot IF Signal
deltaT = sweepTime / freqPoints;
t = deltaT * (0:length(trace) - 1);
fig = figure(1);
clf(fig);
plot(t,trace)
xlabel('Time [s]')
ylabel('Amplitude')

% Plot FFT
c = 3e8; % Speed of light
deltaD = c / (2 * freqSpan); % Resolution of FFT
fig = figure(2);
clf(fig);
y = 20*log(abs(fft(trace-mean(trace))));
x = deltaD * (0:length(y) - 1);
plot(x,y)
xlabel('Distance [m]')
ylabel('Power (dB)')

%% Close serial port
fclose(serialOne);
