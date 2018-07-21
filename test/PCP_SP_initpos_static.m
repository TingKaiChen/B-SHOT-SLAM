clc
clear

frame_s1 = [57 77 97];
frame_s2 = [150 156 161];

sp_06_f57 = [0.984 1 1];
sp_09_f57 = [0.984 0.939 0.930];
sp_12_f57 = [0.989 0.691 0.704];
sp_157_f57 = [0.989 0.588 0.622];

sp_06_f150 = [0.973 0.938 0.985];
sp_09_f150 = [0.973 0.620 0.878];
sp_12_f150 = [0.973 0.620 0.641];
sp_157_f150 = [0.973 0.620 0.641];

% Static case
figure;hold on;
plot(frame_s1, sp_06_f57, '-o');
plot(frame_s1, sp_09_f57, '-^');
plot(frame_s1, sp_12_f57, '-*');
plot(frame_s1, sp_157_f57, '-s');
xlim([50 105]);
ylim([0 1.2]);
xlabel('Frame')
ylabel('Accuracy')
legend('-0.6','-0.9','-1.2','-1.57','Location','southeast')
title('Accuracy of Non-Ground Point: Frame 57-97')
saveas(gcf, './Accuracy_F57.pdf')

% Dynamic case
figure;hold on;
plot(frame_s2, sp_06_f150, '-o');
plot(frame_s2, sp_09_f150, '-^');
plot(frame_s2, sp_12_f150, '-*', 'MarkerSize', 10);
plot(frame_s2, sp_157_f150, '-s');
xlim([147 163]);
ylim([0 1.2]);
xlabel('Frame')
ylabel('Accuracy')
legend('-0.6','-0.9','-1.2','-1.57','Location','southeast')
title('Accuracy of Non-Ground Point: Frame 150-161')
saveas(gcf, './Accuracy_F150.pdf')
