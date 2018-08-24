clc
clear

frame = 1:10;

SR_hitnum = [600	470	418	477	464	463	490	468	426	449];
SR_srcnum = [600	600	600	600	600	600	600	600	600	600];
SR_refnum = [600	600	600	600	600	600	600	600	600	600];
SR_hitrate = [1	0.7833333333	0.6966666667	0.795	0.7733333333	0.7716666667	0.8166666667	0.78	0.71	0.7483333333];
SR_t = [608.707	699.878	585.86	581.026	575.807	579.263	585.004	574.81	574.949	583.779];

ISS_hitnum = [296	209	222	224	214	208	197	214	214	219];
ISS_srcnum = [296	286	300	301	298	284	297	302	287	290];
ISS_refnum = [296	296	286	300	301	298	284	297	302	287];
ISS_hitrate = [1	0.7307692308	0.74	0.7441860465	0.7181208054	0.7323943662	0.6632996633	0.7086092715	0.7456445993	0.7551724138];
ISS_t = [91.1485	60.2688	44.8973	42.915	42.6615	43.2656	43.0941	43.5263	42.6086	42.9487];

% Source number
figure;hold on;
plot(frame, SR_srcnum, '-d');
plot(frame, ISS_srcnum, '-o');
xlim([0 11]);
ylim([0 800]);
xlabel('Frame')
ylabel('Point Number')
legend('SR','ISS')
title('Number of Keypoints')
saveas(gcf, './StaticTest_srcnum.pdf')

% Hit rate
figure;hold on;
plot(frame, SR_hitrate, '-d');
plot(frame, ISS_hitrate, '-o');
xlim([0 11]);
ylim([0 1.1]);
xlabel('Frame')
ylabel('Repeatness Ratio')
legend('SR','ISS')
title('Repeatness Ratio')
saveas(gcf, './StaticTest_hitrate.pdf')

% Time
figure;hold on;
plot(frame, SR_t, '-d');
plot(frame, ISS_t, '-o');
xlim([0 11]);
ylim([0 800]);
xlabel('Frame')
ylabel('Computation Time (ms)')
legend('SR','ISS')
title('Computation Time')
saveas(gcf, './StaticTest_time.pdf')
