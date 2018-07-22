clc
clear

frame = 687:691;

cvs_mean = [390.4 381.4 490.3 454.1 556.4];
cv_mean = [762.1 660.1 415.1 486.7 479.0];
cvsicp_mean = [287.4 368.9 383.1 365.8 344.5];
cvicp_mean = [473.1 501.9 407.3 401.2 472.3];

cvs_std = [271.5 283.9 279.0 311.8 303.1];
cv_std = [300.6 405.8 368.6 367.2 396.8];
cvsicp_std = [295.6 318.5 326.2 340.1 367.3];
cvicp_std = [499.7 482.7 370.2 369.6 403.0];

cvs_num = [84 82 100 93 82];
cv_num = [48 37 53 69 59];

% Mean
figure;hold on;
plot(frame, cvs_mean, '-o');
plot(frame, cv_mean, '-^');
plot(frame, cvsicp_mean, '-*');
plot(frame, cvicp_mean, '-s');
xlim([686 692]);
% ylim([0 1.2]);
xlabel('Frame')
ylabel('(mm)')
legend('Sum','No\_sum','Sum+ICP','No\_sum+ICP','Location','northeast')
title('Mean of Correspondence Distance')
saveas(gcf, './CorrDist_mean.pdf')

% Standard deviation
figure;hold on;
plot(frame, cvs_std, '-o');
plot(frame, cv_std, '-^');
plot(frame, cvsicp_std, '-*');
plot(frame, cvicp_std, '-s');
xlim([686 692]);
ylim([250 530]);
xlabel('Frame')
ylabel('(mm)')
legend('Sum','No\_sum','Sum+ICP','No\_sum+ICP','Location','northeast')
title('Standard Deviation of Correspondence Distance')
saveas(gcf, './CorrDist_std.pdf')

% Correspondence number
figure;hold on;
plot(frame, cvs_num, '-o');
plot(frame, cv_num, '-^');
xlim([686 692]);
ylim([30 110]);
xlabel('Frame')
ylabel('Correspondence number')
legend('Sum','No\_sum','Location','northeast')
title('Correspondence Number')
saveas(gcf, './CorrNum.pdf')
