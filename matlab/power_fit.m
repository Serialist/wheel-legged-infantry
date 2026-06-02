% ========== 1. 准备实验数据 ==========

P_all = oz.power_testpower;
X = [oz.power_testtl, oz.power_testtr, oz.power_testwl, oz.power_testwr];

valid_idx = isfinite(P_all) & all(isfinite(X), 2);
P_all = P_all(valid_idx);
X = X(valid_idx, :);

fitFunc = @(k, X) modelFunc(k, X);

% ========== 3. 设置初始值和边界 ==========
k0 = [1, 1, 1];           % 初始猜测值 [k1, k2, k3]
lb = [-Inf, -Inf, -Inf];           % 下界（根据物理意义设定，如参数非负）
ub = [Inf, Inf, Inf];     % 上界

% ========== 4. 执行最小二乘拟合 ==========
options = optimoptions('lsqcurvefit', ...
    'Display', 'iter', ...           % 显示迭代过程
    'MaxIterations', 30, ...
    'FunctionTolerance', 1e-12, ...
    'StepTolerance', 1e-12);

[k_opt, resnorm, residual, exitflag, output] = lsqcurvefit(...
    fitFunc, k0, X, P_all, lb, ub, options);

% ========== 5. 显示结果 ==========
fprintf('\n========== 拟合结果 ==========\n');
fprintf('k1 = %.6f\n', k_opt(1));
fprintf('k2 = %.6f\n', k_opt(2));
fprintf('k3 = %.6f\n', k_opt(3));
fprintf('残差范数: %.6f\n', resnorm);
fprintf('迭代次数: %d\n', output.iterations);

% ========== 6. 计算拟合优度 ==========
P_fit = fitFunc(k_opt, X);
SS_res = sum(residual.^2);
SS_tot = sum((P_all - mean(P_all)).^2);
R_squared = 1 - SS_res/SS_tot;
fprintf('R² = %.6f\n', R_squared);

% ========== 7. 绘图验证 ==========
figure('Position', [100, 100, 1200, 400]);

% % 子图1: 观测值 vs 拟合值
% subplot(1, 3, 1);
% scatter(P_all, P_fit, 25, 'b', 'filled');
% hold on;
% plot([min(P_all), max(P_all)], [min(P_all), max(P_all)], 'r--', 'LineWidth', 2);
% xlabel('观测值 P_{all}');
% ylabel('拟合值 P_{fit}');
% title('观测值 vs 拟合值');
% grid on;
% axis equal;
% 
% % 子图2: 残差分布
% subplot(1, 3, 2);
% histogram(residual, 20, 'FaceColor', [0.3 0.6 0.9]);
% xlabel('残差');
% ylabel('频数');
% title('残差分布');
% grid on;
% 
% % 子图3: 残差 vs 拟合值
% subplot(1, 3, 3);
% scatter(P_fit, residual, 25, 'g', 'filled');
% hold on;
% yline(0, 'r--', 'LineWidth', 2);
% xlabel('拟合值');
% ylabel('残差');
% title('残差图');
% grid on;

% sgtitle('最小二乘拟合结果验证');

% 主图：P_all vs P_fit
plot(oz.Time(valid_idx), P_all(:), 'b-', 'LineWidth', 1, 'DisplayName', 'P_{all} 实际');
hold on;
plot(oz.Time(valid_idx), P_fit(:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'P_{fit} 拟合');

%% ========== 模型函数定义 ==========
function P = modelFunc(k, X)
    % k: [k1, k2, k3]
    % X: N×4 矩阵，每列为 [tau_l, omega_l, tau_r, omega_r]
    
    k1 = k(1);
    k2 = k(2);
    k3 = k(3);
    
    tau_l = X(:, 1);
    tau_r = X(:, 2);
    omega_l = X(:, 3);
    omega_r = X(:, 4);
    
    % 计算模型输出
    P = tau_l.*omega_l + tau_r.*omega_r ...
      + k1*(abs(omega_l) + abs(omega_r)) ...
      + k2*(tau_l.^2 + tau_r.^2) ...
      + k3;
end