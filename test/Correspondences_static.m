clc
clear

%% Structural scene
frame = 687:691;

cvsn_med = [550.5 196.3 666.9 320.3 288.9];
cvs_med = [273 270.7 399.7 376.7 466.1];
cv_med = [735.9 578.2 239.5 343.5 283];
cvsnicp_med = [231.5 202.1 287.6 188.5 233.3];
cvsicp_med = [174.3 232.5 256 239.8 197.4];
cvicp_med = [252.9 263.7 249.2 247 287.7];

cvsn_mean = [590.4 291 710.2 430.9 367.3];
cvs_mean = [390.4 381.4 490.3 454.1 556.4];
cv_mean = [762.1 660.1 415.1 486.7 479.0];
cvsnicp_mean = [327.7 301.3 366.9 330.1 336.9];
cvsicp_mean = [287.4 368.9 383.1 365.8 344.5];
cvicp_mean = [473.1 501.9 407.3 401.2 472.3];

cvsn_std = [330.4 266.9 262 329.5 278.7];
cvs_std = [271.5 283.9 279.0 311.8 303.1];
cv_std = [300.6 405.8 368.6 367.2 396.8];
cvsnicp_std = [251.6 265.2 308.5 314.4 292.3];
cvsicp_std = [295.6 318.5 326.2 340.1 367.3];
cvicp_std = [499.7 482.7 370.2 369.6 403.0];

cvsn_num = [84 99 100 101 88];
cvs_num = [84 82 100 93 82];
cv_num = [48 37 53 69 59];

% Median
figure;hold on;
plot(frame, cvsn_med, '-d');
plot(frame, cvs_med, '-o');
plot(frame, cv_med, '-^');
plot(frame, cvsnicp_med, '-p');
plot(frame, cvsicp_med, '-*');
plot(frame, cvicp_med, '-s');
xlim([686 692]);
ylim([100 800]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','northeast')
title('Median of Correspondence Distance in Structural Scene')
saveas(gcf, './CorrDist_med_structural_F687.pdf')

% Mean
figure;hold on;
plot(frame, cvsn_mean, '-d');
plot(frame, cvs_mean, '-o');
plot(frame, cv_mean, '-^');
plot(frame, cvsnicp_mean, '-p');
plot(frame, cvsicp_mean, '-*');
plot(frame, cvicp_mean, '-s');
xlim([686 692]);
ylim([100 800]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','northeast')
title('Mean of Correspondence Distance in Structural Scene')
saveas(gcf, './CorrDist_mean_structural_F687.pdf')

% Standard deviation
figure;hold on;
plot(frame, cvsn_std, '-d');
plot(frame, cvs_std, '-o');
plot(frame, cv_std, '-^');
plot(frame, cvsnicp_std, '-p');
plot(frame, cvsicp_std, '-*');
plot(frame, cvicp_std, '-s');
xlim([686 692]);
ylim([220 530]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','northeast')
title('STD. of Correspondence Distance in Structural Scene')
saveas(gcf, './CorrDist_std_structural_F687.pdf')

% Correspondence number
figure;hold on;
plot(frame, cvsn_num, '-d');
plot(frame, cvs_num, '-o');
plot(frame, cv_num, '-^');
xlim([686 692]);
ylim([30 110]);
xlabel('Frame')
ylabel('Correspondence number')
legend('CVSN','CVS','CV','Location','southeast')
title('Correspondence Number in Structural Scene')
saveas(gcf, './CorrNum_structural_F687.pdf')





%% Structural scene
frame = 281:285;

cvsn_med = [118 154.3 197.3 400.7 241];
cvs_med = [120.7 334.2 304 236.5 372.4];
cv_med = [47.7 175.7 377.3 248.4 355.3];
cvsnicp_med = [255.4 144.7 141 197.8 139.7];
cvsicp_med = [306.6 366 228.8 438 207.8];
cvicp_med = [181.1 183.4 119.8 276.8 191.6];

cvsn_mean = [126.4 186.8 211.3 422.6 280.8];
cvs_mean = [165 374.4 392.4 294.9 376.1];
cv_mean = [87.9 248.7 424.4 291 398.2];
cvsnicp_mean = [237.7 207.6 187.1 256 224.3];
cvsicp_mean = [344.9 381.4 294.8 439.8 266.4];
cvicp_mean = [206.6 246.8 178.6 303.1 255.2];

cvsn_std = [114.4 158.6 123.5 218.6 206.1];
cvs_std = [136 211 269.5 215.8 224.5];
cv_std = [133.2 211.8 142.8 164.4 208.7];
cvsnicp_std = [134.4 171.8 139.2 197.6 235.3];
cvsicp_std = [197 223.1 218.2 201.2 197.2];
cvicp_std = [116.3 211.2 186.1 162 244.8];

cvsn_num = [133 154 140 133 151];
cvs_num = [92 108 109 104 105];
cv_num = [114 126 133 122 116];

% Median
figure;hold on;
plot(frame, cvsn_med, '-d');
plot(frame, cvs_med, '-o');
plot(frame, cv_med, '-^');
plot(frame, cvsnicp_med, '-p');
plot(frame, cvsicp_med, '-*');
plot(frame, cvicp_med, '-s');
xlim([280 286]);
ylim([-30 470]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','southeast')
title('Median of Correspondence Distance in Structureless Scene')
saveas(gcf, './CorrDist_med_structureless_F281.pdf')

% Mean
figure;hold on;
plot(frame, cvsn_mean, '-d');
plot(frame, cvs_mean, '-o');
plot(frame, cv_mean, '-^');
plot(frame, cvsnicp_mean, '-p');
plot(frame, cvsicp_mean, '-*');
plot(frame, cvicp_mean, '-s');
xlim([280 286]);
ylim([-30 470]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','southeast')
title('Mean of Correspondence Distance in Structureless Scene')
saveas(gcf, './CorrDist_mean_structureless_F281.pdf')

% Standard deviation
figure;hold on;
plot(frame, cvsn_std, '-d');
plot(frame, cvs_std, '-o');
plot(frame, cv_std, '-^');
plot(frame, cvsnicp_std, '-p');
plot(frame, cvsicp_std, '-*');
plot(frame, cvicp_std, '-s');
xlim([280 286]);
ylim([90 290]);
xlabel('Frame')
ylabel('(mm)')
legend('CVSN','CVS','CV','CVSN+ICP','CVS+ICP','CV+ICP','Location','southeast')
title('STD. of Correspondence Distance in Structureless Scene')
saveas(gcf, './CorrDist_std_structureless_F281.pdf')

% Correspondence number
figure;hold on;
plot(frame, cvsn_num, '-d');
plot(frame, cvs_num, '-o');
plot(frame, cv_num, '-^');
xlim([280 286]);
ylim([80 160]);
xlabel('Frame')
ylabel('Correspondence number')
legend('CVSN','CVS','CV','Location','southeast')
title('Correspondence Number in Structureless Scene')
saveas(gcf, './CorrNum_structureless_F281.pdf')
