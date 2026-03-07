% 本程序用于求解LQR反馈矩阵lqr_k(L0) 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 针对五段theta值所作的多项式拟合

clc;
clear;
py.lqr_k_extract.file_clear();

L0s = 0.12:0.01:0.32;          % L0变化范围
theta_list = [0 15 30 45 60];
Ks_by_theta = cell(length(theta_list), 1);  % 存储每个theta对应的K矩阵
poly_coeffs = cell(length(theta_list), 1);  % 存储每个theta对应的多项式系数

% 对每个theta值分别计算
for theta_idx = 1:length(theta_list)
    current_theta = theta_list(theta_idx);
    Ks = zeros(2, 6, length(L0s)); % 存放当前theta下不同L0对应的K
    
    % 对每个L0计算K矩阵
    for L_idx = 1:length(L0s)
        
        % 所需符号量
        syms theta dtheta ddtheta;
        syms x x1 x2;
        syms phi phi1 phi2;
        syms T Tp Nf t;
        
        % 机器人结构参数
        R = 0.0775;
        L = L0s(L_idx) / 2;
        Lm = L0s(L_idx) / 2;
        mw = 0.334 * 2;
        l = 0.004;
        mp = 1.482 * 2;
        M = (17.5 + 0.68 - mp * 2 - mw * 2);
        Iw = 0.5 * mw * R ^ 2;
        Ip = mp * ((L + Lm) ^ 2 + 0.144 ^ 2) / 12.0;
        Im = 0.23203539;
        g = 9.8;
    
        % 进行物理计算
        Nm = M * (x2 + (L + Lm) * (ddtheta * cos(theta) - dtheta ^ 2 * sin(theta)) - l * (phi2 * cos(phi) - phi1 ^ 2 * sin(phi)));
        Pm = M * g + M * ((L + Lm) * (-dtheta ^ 2 * cos(theta) - ddtheta * sin(theta)) - l * (phi1 ^ 2 * cos(phi) + phi2 * sin(phi)));
        N = Nm + mp * (x2 + L * (ddtheta * cos(theta) - dtheta ^ 2 * sin(theta)));
        P = Pm + mp * g + mp * L * (-dtheta ^ 2 * cos(theta) - ddtheta * sin(theta));
        
        equ1 = x2 - (T - N * R) / (Iw / R + mw * R);
        equ2 = (P * L + Pm * Lm) * sin(theta) - (N * L + Nm * Lm) * cos(theta) - T + Tp - Ip * ddtheta;
        equ3 = Tp + Nm * l * cos(phi) + Pm * l * sin(phi) - Im * phi2;
    
        [x2_sol, ddtheta_sol, phi2_sol] = solve(equ1, equ2, equ3, x2, ddtheta, phi2);
        
        % 求得雅克比矩阵，然后得到状态空间方程
        Ja = jacobian([dtheta; ddtheta_sol; x1; x2_sol; phi1; phi2_sol], [theta dtheta x x1 phi phi1]);
        Jb = jacobian([dtheta; ddtheta_sol; x1; x2_sol; phi1; phi2_sol], [T Tp]);

        % 代入当前theta值，注意单位转换为弧度
        theta_rad = deg2rad(current_theta);
        
        % 代入数值并转换为数值矩阵
        A = vpa(subs(Ja, [theta dtheta x x1 phi phi1 Tp T], [theta_rad 0 0 0 0 0 0 0]));
        B = vpa(subs(Jb, [theta dtheta x x1 phi phi1 Tp T], [theta_rad 0 0 0 0 0 0 0]));

        % 离散化
        [G, H] = c2d(eval(A), eval(B), 0.005);
        
        % 权重 Q R
        Q = diag([1000.0, 0.5, 150.0, 10.0, 10000.0, 0.5]);
        R_matrix = diag([1.0000, 0.2500]);  % 重命名避免与R半径冲突
    
        % 求解 LQR
        Ks(:,:,L_idx) = dlqr(G, H, Q, R_matrix);
    end
    
    % 当前 theta 的所有 K
    Ks_by_theta{theta_idx} = Ks;
    
    % 关于 L0 多项式拟合
    syms L0;
    K_poly = sym(zeros(2, 6));  % 存储多项式表达式
    coeffs_cell = cell(2, 6);   % 存储系数
    
    % 存储当前theta的多项式系数
    poly_coeffs{theta_idx} = coeffs_cell;
    
    for i = 1:2
        for j = 1:6
            % 提取当前元素在所有L0下的值
            element_values = squeeze(Ks(i, j, :));
            
            % 3 阶多项式拟合
            p = polyfit(L0s', element_values, 3);
            
            % 存储系数
            coeffs_cell{i, j} = p;
            
            % 构建多项式表达式
            K_poly(i, j) = poly2sym(p, L0);
        end
    end


    % 输出到m函数
    matlabFunction(K_poly, 'File', 'lqr_k');
    
    py.lqr_k_extract.extract_k(theta_idx)

    % 代入L0 = 0.20打印矩阵K 
    len = 0.20;
    fprintf("len = %.2f, theta = %d:\n", len, theta_list(theta_idx));
    disp(vpa(subs(K_poly, L0, 0.20)));
end
