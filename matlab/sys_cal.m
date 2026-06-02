% 本程序用于求解LQR反馈矩阵lqr_k(L0) 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 对于不同的K，对L0进行拟合，得到lqr_k

clc;
clear;

py.lqr_k_extract.fclear()

L0s = 0.12:0.01:0.32;          % L0变化范围
Ks = zeros(2, 6, length(L0s)); % 存放不同L0对应的K
theta_list = 0;

K_data = zeros(2, 6, length(theta_list), length(L0s), "double"); % 储存所有K

for theta_step = 1:length(theta_list)
    
    theta_t = deg2rad(theta_list(theta_step));
    
    for step=1:length(L0s)
        
        % 所需符号量
        syms theta dtheta ddtheta;
        syms x x1 x2;
        syms phi phi1 phi2;
        syms T Tp Nf t;
        
        % 机器人结构参数
        R = 0.068;
        L = L0s(step) / 2;
        Lm = L0s(step) / 2;
        mw = 0.334 + 0.179;
        l = 0.036; % 机体质心到髋关机中心距离
        mp = 1;
        M = (23.4 - mp - mw) / 2;
        Iw = 0.000956;
        Ip = mp * ((L + Lm) ^ 2 + 0.144 ^ 2) / 12.0;
        Im = 0.353588749;
        g = 9.8;
    
        % 进行物理计算
        Nm = M * (x2 + (L + Lm) * (ddtheta  * cos(theta) - dtheta  ^ 2 * sin(theta)) - l * (phi2 * cos(phi) - phi1 ^ 2 * sin(phi)));
        Pm = M * g + M * ((L + Lm) * (-dtheta  ^ 2 * cos(theta) - ddtheta  * sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
        N = Nm + mp * (x2 + L * (ddtheta  * cos(theta) - dtheta  ^ 2 * sin(theta)));
        P = Pm + mp * g + mp * L * (-dtheta  ^ 2 * cos(theta) - ddtheta  * sin(theta));
        
        equ1 = x2 - (T - N * R) / (Iw / R + mw * R);
        equ2 = (P * L + Pm * Lm) * sin(theta) - (N * L + Nm * Lm) * cos(theta) - T + Tp - Ip * ddtheta;
        equ3 = Tp + Nm * l * cos(phi) + Pm * l * sin(phi) - Im * phi2;
    
        [x2,ddtheta ,phi2] = solve(equ1, equ2, equ3, x2, ddtheta, phi2);
        
        % 求得雅克比矩阵，然后得到状态空间方程
        Ja = jacobian( [dtheta; ddtheta; x1; x2; phi1; phi2], [theta dtheta  x x1 phi phi1]);
        Jb = jacobian( [dtheta; ddtheta; x1; x2; phi1; phi2], [T Tp]);

        % 代入数值并转换为数值矩阵
        A = vpa(subs(Ja, [theta dtheta x x1 phi phi1 Tp T], [0 0 0 0 0 0 0 0]));
        B = vpa(subs(Jb, [theta dtheta x x1 phi phi1 Tp T], [0 0 0 0 0 0 0 0]));

        % 离散化
        [G, H] = c2d(double(A), double(B), 0.001);
        
        % 定义权重矩阵Q, R
        % PSO 参数
        % Q = diag([1000, 5, 150, 10, 10000, 0.5]);
        % R = diag([1 0.25]);

        % 新参数仿真试出来的就是不太好使   
        % Q = diag([2000, 75, 25, 25, 8000, 5]);
        % R_ = diag([80 5]);

        % 这版不错，比较均衡
        % Q = diag([5000, 100, 200, 20, 8000, 5]);
        % R_ = diag([300 10]);

        % 这个参数非常好，我就不改了(ᗜ‸ᗜ)
        % 不过还是有一点点问题
        % 位移可以再改一点
        % 其实pitch和theta的K权重差不多是正确的（k21和k25）
        % 但是当theta过大，可能出现pitch倾翻，严重甚至可能翻车？建议加入快倒地的检测
        % 不过说不定pitch是可以把自己拉回来的，可能加大Q66（dpitch）有用
        % @date 2026-03-10
        % Q = diag([6000, 80, 4000, 0.01, 8000, 20]);
        % R_ = diag([50 5]);

        % Q = diag([6000, 80, 900, 300, 8000, 20]);
        % R_ = diag([40 10]);

        Q = diag([6000, 20, 2000, 500, 8000, 20]);
        R_ = diag([60 10]);
    
        % 求解反馈矩阵K
        K_val = dlqr(G, H, Q, R_);
        K_val_numeric = double(real(K_val)); 
        Ks(:,:,step) = K_val_numeric;
        K_data(:,:,theta_step, step) = K_val_numeric;
    
    end
    
    % 对K的每个元素关于L0进行拟合
    K = sym('K', [2 6]);
    syms L0;
    
    for x = 1:2
        for y = 1:6
            p = polyfit(L0s, reshape(Ks(x,y,:), 1,length(L0s)), 3);
            K(x, y) = p(1) * L0 ^ 3 + p(2) * L0 ^ 2 + p(3) * L0 + p(4);
        end
    end
    
    % 输出到m函数
    matlabFunction(K, 'File', 'lqr_k');
    
    py.lqr_k_extract.extrect_coef(theta_step)
    
    % 代入L0 = 0.20打印矩阵K 
    len = 0.20;
    fprintf("len = %.2f, theta = %d:\n", len, theta_list(theta_step));
    disp(eval(vpa(subs(K, L0, 0.20))));
end

py.lqr_k_extract.ftail();

disp("计算完成");

%% 显示K矩阵变化图像
state_names = {'\theta', 'd\theta', 'x', 'dx', 'phi', 'dphi'};
control_names = {'T (Wheel Torque)', 'Tp (Swing Torque)'};
colors = lines(length(theta_list)); % 自动生成对比颜色

% for u_idx = 1:2
%     figure('Name', ['Control Gain for ' control_names{u_idx}], 'Color', 'w');
%     for x_idx = 1:6
%         subplot(2, 3, x_idx);
%         hold on; grid on;
%         for t_idx = 1:length(theta_list)
%             plot(L0s, squeeze(K_data(u_idx, x_idx, t_idx, :)), 'LineWidth', 1.5, ...
%                 'Color', colors(t_idx, :), 'DisplayName', [num2str(theta_list(t_idx)) '°']);
%         end
%         title(['K(' num2str(u_idx) ',' num2str(x_idx) ') - ' state_names{x_idx}]);
%         xlabel('L0 (m)'); ylabel('Gain Value');
%         if x_idx == 1, legend('Location', 'best'); end
%     end
% end
% disp("图像生成");
