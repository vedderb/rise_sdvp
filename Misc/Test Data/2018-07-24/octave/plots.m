clear all;
close all;
clc;

fi_04_diff_1 = [
0.4959991633057443
0.29221439047384623
0.29247750340838186
0.3818522881953188
1.0383722357613352
0.37172000215215834
0.2651401327600188
0.510858111416464
]';

fi_04_diff_2 = [
0.516707460755114
0.27073243248639567
0.32241688851547645
0.39604006110493306
1.052817515051872
0.378093586298418
0.2720571998679644
0.47496374598489144
]';

fi_04_diff_3 = [
0.49552422746017233
0.2465594046066813
0.2892000172890738
0.3633095099223211
1.0176149566510888
0.3687629048589366
0.233872486624651
0.5057531512506873
]';

fi_04_nf_diff_1 = [
0.4935148731294721
0.3125882275454412
0.21833868186833255
0.37350535471395846
0.40975126601390477
0.3692638216776746
0.27024289074829805
0.5019404845198276
]';

fi_04_nf_diff_2 = [
0.5022471304049375
0.2850821109785695
0.3065606954585011
0.3947804453110665
0.4151965558624045
0.3647261575483703
0.24788771651697777
0.4886496188476934
]';

fi_04_nf_diff_3 = [
0.5018850266744369
0.27669654858707443
0.24805563891998014
0.39034090228926716
0.40782707119562445
0.3793007382012334
0.28222808152272744
0.496280021761908
]';

xaxis = 1:8;
axsize = [xaxis(1), xaxis(end), 0, 1.1];

avg_fi = (fi_04_diff_1 + fi_04_diff_2 + fi_04_diff_3) / 3;
avg_no_fi = (fi_04_nf_diff_1 + fi_04_nf_diff_2 + fi_04_nf_diff_3) / 3;

f = figure
hold on;
plot(xaxis, fi_04_diff_1);
plot(xaxis, abs(avg_fi - fi_04_diff_1), "--");
plot(xaxis, fi_04_diff_2, 'r');
plot(xaxis, abs(avg_fi - fi_04_diff_2), "r--");
plot(xaxis, fi_04_diff_3, 'k');
plot(xaxis, abs(avg_fi - fi_04_diff_3), "k--");
grid on;
axis(axsize)
legend("Run 1   ", "Diff 1", "Run 2", "Diff 2", "Run 3", "Diff 3")
xlabel("Trajectory Part")
ylabel("Error / Deviation (m)")

set (f, "papersize", [6.4, 4.8])
set (f, "paperposition", [0, 0, 6.4, 4.8]) 
print(f, "fi4_runs.pdf", "-dpdf")

f = figure;
hold on;
plot(xaxis, fi_04_nf_diff_1);
plot(xaxis, abs(avg_no_fi - fi_04_nf_diff_1), "--");
plot(xaxis, fi_04_nf_diff_2, 'r');
plot(xaxis, abs(avg_no_fi - fi_04_nf_diff_2), "r--");
plot(xaxis, fi_04_nf_diff_3, 'k');
plot(xaxis, abs(avg_no_fi - fi_04_nf_diff_3), "k--");
grid on;
axis(axsize)
legend("Run 1   ", "Diff 1", "Run 2", "Diff 2", "Run 3", "Diff 3")
xlabel("Trajectory Part")
ylabel("UWB Error (m)")

set (f, "papersize", [6.4, 4.8])
set (f, "paperposition", [0, 0, 6.4, 4.8]) 
print(f, "fi4_runs_nf.pdf", "-dpdf")

max([abs(avg_fi - fi_04_diff_1) abs(avg_fi - fi_04_diff_2) abs(avg_fi - fi_04_diff_3)])
max([abs(avg_no_fi - fi_04_nf_diff_1) abs(avg_no_fi - fi_04_nf_diff_2) abs(avg_no_fi - fi_04_nf_diff_3)])

close all
