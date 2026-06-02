% 修改后的 sys_cal.m：输出 K 矩阵所有 12 个变量的 2D 曲线图
clc;
clear;

% 清理并准备导出文件
py.lqr_k_extract.fclear()

L0s = 0.12:0.01:0.32;          % L0 变化范围
theta_list = [0 15 30 45 60];  % 角度步进列表
colors = lines(length(theta_list)); % 自动生成对比颜色

% 存储所有计算得到的 K 矩阵 [u, x, theta, L0]
K_data = zeros(2, 6, length(theta_list), length(L0s)); 

for theta_step = 1:length(theta_list)
    theta_t = deg2rad(theta_list(theta_step));
    Ks = zeros(2, 6, length(L0s)); % 临时存储当前角度下的 K
    
    for step = 1:length(L0s)
        % --- 动力学建模部分（保持原逻辑不变） ---
        syms theta dtheta ddtheta x x1 x2 phi phi1 phi2 T Tp;
        R = 0.0775; L = L0s(step)/2; Lm = L0s(step)/2; mw = 0.334*2; l = 0.2; mp = 1.482*2;
        M = (17.5 + 0.68 - mp*2 - mw*2); Iw = 0.5 * mw * R^2;
        Ip = mp * ((L + Lm)^2 + 0.144^2) / 12.0; Im = 0.23203539; g = 9.8;
        
        Nm = M*(x2+(L+Lm)*(ddtheta*cos(theta)-dtheta^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
        Pm = M*g+M*((L+Lm)*(-dtheta^2*cos(theta)-ddtheta*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
        N = Nm+mp*(x2+L*(ddtheta*cos(theta)-dtheta^2*sin(theta)));
        P = Pm+mp*g+mp*L*(-dtheta^2*cos(theta)-ddtheta*sin(theta));
        
        [x2_r, ddtheta_r, phi2_r] = solve(x2-(T-N*R)/(Iw/R+mw*R), ...
            (P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*ddtheta, ...
            Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2, x2, ddtheta, phi2);
        
        Ja = jacobian([dtheta; ddtheta_r; x1; x2_r; phi1; phi2_r], [theta dtheta x x1 phi phi1]);
        Jb = jacobian([dtheta; ddtheta_r; x1; x2_r; phi1; phi2_r], [T Tp]);
        
        A = eval(subs(Ja, [theta dtheta x x1 phi phi1 Tp T], [theta_t 0 0 0 0 0 0 0]));
        B = eval(subs(Jb, [theta dtheta x x1 phi phi1 Tp T], [theta_t 0 0 0 0 0 0 0]));
        
        [G, H] = c2d(A, B, 0.005);
        Q = diag([3000, 75, 50, 25, 8000, 5]); % 状态权重
        R_mat = diag([80 5]); % 控制权重
        
        K_val = dlqr(G, H, Q, R_mat);
        Ks(:,:,step) = K_val;
        K_data(:,:,theta_step, step) = K_val;
    end
    
    % --- 导出逻辑（保持原逻辑不变） ---
    K_sym = sym('K', [2 6]); syms L0_sym;
    for r = 1:2
        for c = 1:6
            p = polyfit(L0s, reshape(Ks(r,c,:), 1, length(L0s)), 3);
            K_sym(r, c) = p(1)*L0_sym^3 + p(2)*L0_sym^2 + p(3)*L0_sym + p(4);
        end
    end
    matlabFunction(K_sym, 'File', 'lqr_k');
    % py.lqr_k_extract.extrect_coef(theta_step);
end

%% --- 绘图部分：展示所有 12 个变量的图像 ---
state_names = {'\theta', 'd\theta', 'x', 'dx', 'phi', 'dphi'};
control_names = {'T (Wheel Torque)', 'Tp (Swing Torque)'};

for u_idx = 1:2
    figure('Name', ['Control Gain for ' control_names{u_idx}], 'Color', 'w');
    for x_idx = 1:6
        subplot(2, 3, x_idx);
        hold on; grid on;
        for t_idx = 1:length(theta_list)
            plot(L0s, squeeze(K_data(u_idx, x_idx, t_idx, :)), 'LineWidth', 1.5, ...
                'Color', colors(t_idx, :), 'DisplayName', [num2str(theta_list(t_idx)) '°']);
        end
        title(['K(' num2str(u_idx) ',' num2str(x_idx) ') - ' state_names{x_idx}]);
        xlabel('L0 (m)'); ylabel('Gain Value');
        if x_idx == 1, legend('Location', 'best'); end
    end
end

disp("计算完成，图像已生成。");