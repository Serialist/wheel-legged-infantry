%% ========== 1. 放入你的数据：t = 时间, tt = 信号 ==========
% clear; clc; close all;

% 这里有时间t和数据tt

% ========== 自动计算采样频率 ==========
Fs = 1/mean(diff(t));  % 自动算采样率(Hz)，不用你手动输入
N = length(tt);

%% ========== 2. FFT 频率分析 ==========
f = Fs*(0:(N/2))/N;    % 频率轴
Y = fft(tt);
P = abs(Y/N);
P = P(1:N/2+1);        % 单边频谱

% 画图：原始信号 + 频谱
figure('Position',[100,100,800,500])
subplot(2,1,1)
plot(t, tt, 'b', 'LineWidth', 1);
xlabel('时间 t'); ylabel('信号值 tt');
title('原始时间序列'); grid on;

subplot(2,1,2)
plot(f, P, 'r', 'LineWidth', 1.2);
xlabel('频率 (Hz)'); ylabel('幅值');
title('频率特性（频谱图）→ 看这里确定截止频率');
xlim([0 Fs/2]); grid on;

%% ========== 3. 确定低通截止频率 cutfreq ==========
% 看上面的频谱图：
% 低频幅值高 = 有效信号
% 高频幅值骤降/很小 = 噪声
% cutfreq 取两者分界点即可

cutfreq = 30;  % 【你根据频谱图修改这里】

fprintf('==================================================\n');
fprintf('采样频率 Fs = %.2f Hz\n', Fs);
fprintf('推荐低通截止频率 cutfreq = %.2f Hz\n', cutfreq);
fprintf('==================================================\n');

%% ========== 4. 低通滤波 ==========
order = 100;
b = fir1(order, cutfreq/(Fs/2), 'low');
tt_filt = filtfilt(b, 1, tt);  % 零相位滤波（最好用）

%% ========== 5. 滤波效果对比 ==========
figure('Position',[100,100,800,300])
plot(t, tt, 'b--', 'LineWidth',1); hold on;
plot(t, tt_filt, 'k-', 'LineWidth',1.5);
xlabel('时间 t'); ylabel('信号值');
legend('原始信号','低通滤波后');
title('滤波效果对比'); grid on;