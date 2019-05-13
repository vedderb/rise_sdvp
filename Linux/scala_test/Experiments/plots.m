clear all;
close all;
clc;

filename = "0/filename_2019-05-13-14-02-38";
run([filename ".m"]);

%xaxis = log_data(1:end, 1)';
xaxis = 1:length(log_data(1:end, 2));

f = figure
hold on;
plot(xaxis, log_data(1:end, 2)');
grid on;
xlabel("Accumulated Distance")
ylabel("Error / Deviation (m)")

set (f, "papersize", [6.4, 4.8])
set (f, "paperposition", [0, 0, 6.4, 4.8]) 
print(f, [filename ".pdf"], "-dpdf")

waitfor(f)

