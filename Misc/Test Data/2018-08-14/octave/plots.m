clear all;
close all;
clc;

fi_00_diff_1 = [
0.4563453626366762
0.5257697309659422
0.7344664730265087
0.7417134284883858
1.7688912487770443
]';

fi_00_diff_2 = [
0.4701361930334735
0.5049436503214801
0.6916366459348444
0.7078021192395552
1.4645321573799617
]';

fi_00_diff_3 = [
0.4643317779346893
0.4865963830527289
0.6735014773554691
0.689852621941817
1.734458716718279
]';

fi_00_nf_diff_1 = [
0.4617906885159218
0.5205157442383458
0.531388831271411
0.5547361895531954
0.5440671741614271
]';

fi_00_nf_diff_2 = [
0.5068296064754
0.5311830287198777
0.5151184426906084
0.520708642140694
0.42426158204579834
]';

fi_00_nf_diff_3 = [
0.5059059299909309
0.5293777857069508
0.5204883187930405
0.547898859279703
0.43842309473840413
]';

xaxis = 1:length(fi_00_diff_1);
axsize = [xaxis(1), xaxis(end), 0, 2.1];

avg_fi = (fi_00_diff_1 + fi_00_diff_2 + fi_00_diff_3) / 3;
avg_no_fi = (fi_00_nf_diff_1 + fi_00_nf_diff_2 + fi_00_nf_diff_3) / 3;

f = figure
hold on;
plot(xaxis, fi_00_diff_1);
plot(xaxis, abs(avg_fi - fi_00_diff_1), "--");
plot(xaxis, fi_00_diff_2, 'r');
plot(xaxis, abs(avg_fi - fi_00_diff_2), "r--");
plot(xaxis, fi_00_diff_3, 'k');
plot(xaxis, abs(avg_fi - fi_00_diff_3), "k--");
grid on;
axis(axsize)
legend("Run 1   ", "Diff 1", "Run 2", "Diff 2", "Run 3", "Diff 3")
xlabel("Trajectory Part")
ylabel("Error / Deviation (m)")

set (f, "papersize", [6.4, 4.8])
set (f, "paperposition", [0, 0, 6.4, 4.8]) 
print(f, "fi0_runs.pdf", "-dpdf")

f = figure;
hold on;
plot(xaxis, fi_00_nf_diff_1);
plot(xaxis, abs(avg_no_fi - fi_00_nf_diff_1), "--");
plot(xaxis, fi_00_nf_diff_2, 'r');
plot(xaxis, abs(avg_no_fi - fi_00_nf_diff_2), "r--");
plot(xaxis, fi_00_nf_diff_3, 'k');
plot(xaxis, abs(avg_no_fi - fi_00_nf_diff_3), "k--");
grid on;
axis(axsize)
legend("Run 1   ", "Diff 1", "Run 2", "Diff 2", "Run 3", "Diff 3")
xlabel("Trajectory Part")
ylabel("UWB Error (m)")

set (f, "papersize", [6.4, 4.8])
set (f, "paperposition", [0, 0, 6.4, 4.8]) 
print(f, "fi0_runs_nf.pdf", "-dpdf")

max([abs(avg_fi - fi_00_diff_1) abs(avg_fi - fi_00_diff_2) abs(avg_fi - fi_00_diff_3)])
max([abs(avg_no_fi - fi_00_nf_diff_1) abs(avg_no_fi - fi_00_nf_diff_2) abs(avg_no_fi - fi_00_nf_diff_3)])

close all
