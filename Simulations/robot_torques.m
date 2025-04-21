% robot_kinematics_full.m
% Full 4DOF Robot: Kinematic Modelling, FK, IK, GUI + Interactive Torque Control

clc; clear; close all;

%% Robot Definition: Calculate DOF using Gruebler's Formula
n = 5; j = 4; fi = 1;
DOF = 6 * (n - 1 - j) + j * fi;
fprintf('Calculated DOF using Gruebler''s formula: %d\n', DOF);

%% DH Parameters for Custom Robot (a, alpha, d, theta placeholder)
DH = [  0       pi/2     130     0;
       120       0        0      0;
        90       0        0      0;
        28       0        0      0];

%% Masses (kg)
link_masses = [0.0325, 0.0285, 0.012];     % Links 1–3
servo_masses = [0.060, 0.060, 0.015];      % Servos on joints 2–4

%% Gravity and efficiency
E = 0.9;   % efficiency factor for joint 1
g = 9.81;  % gravity

%% GUI Setup
fig = figure('Name', '4DOF Robot Control Panel', 'Position', [100, 100, 1100, 600]);
slider_vals = zeros(1, DOF);
sliders = gobjects(1, DOF);
labels = {'Base (θ1)', 'Shoulder (θ2)', 'Elbow (θ3)', 'Gripper (θ4)'};
slider_limits = [0 360; 0 180; 0 180; 0 75];

for i = 1:DOF
    idx = DOF - i + 1;
    uicontrol('Style', 'text', 'Position', [50, 540 - (i-1)*100, 100, 20], 'String', labels{idx});
    sliders(idx) = uicontrol('Style', 'slider', 'Min', slider_limits(idx,1), 'Max', slider_limits(idx,2), ...
        'Value', slider_limits(idx,1), 'Position', [160, 540 - (i-1)*100, 400, 20], ...
        'Callback', @(src,~) update_robot());
end

% Payload slider
uicontrol('Style', 'text', 'Position', [50, 100, 100, 20], 'String', 'Payload (g)');
payload_slider = uicontrol('Style', 'slider', 'Min', 0, 'Max', 100, 'Value', 5, ...
    'SliderStep', [0.01 0.1], 'Position', [160, 100, 400, 20], ...
    'Callback', @(src,~) update_robot());

% Axes
ax3D = axes('Units', 'pixels', 'Position', [600, 300, 450, 250]);
title(ax3D, '3D Robot Arm'); xlabel(ax3D, 'X (mm)'); ylabel(ax3D, 'Y (mm)'); zlabel(ax3D, 'Z (mm)');
grid(ax3D, 'on'); axis(ax3D, 'equal'); view(ax3D, 135, 30);

ax2D = axes('Units', 'pixels', 'Position', [600, 30, 450, 250]);
title(ax2D, '2D Side View (XZ-Plane)'); xlabel(ax2D, 'X (mm)'); ylabel(ax2D, 'Z (mm)');
grid(ax2D, 'on'); axis(ax2D, 'equal');

update_robot();

%% Update Robot
function update_robot()
    g = 9.81;
    L1 = 0.120; L2 = 0.090; L3 = 0.028;
    m1 = 0.0325; m2 = 0.0285; m3 = 0.012;
    m2s = 0.060; m3s = 0.060; mg = 0.015;

    fig_data = get(gcf, 'UserData');
    if isempty(fig_data)
        sliders = findall(gcf, 'Style', 'slider');
    else
        sliders = fig_data.sliders;
    end

    % Get joint angles
    theta_deg = zeros(1, 4);
    for j = 1:4
        theta_deg(j) = get(sliders(j), 'Value');
    end
    theta_rad = deg2rad(theta_deg);

    % Get payload from slider (g -> kg)
    payload = get(findobj(gcf, 'Style', 'slider', 'Position', [160, 100, 400, 20]), 'Value') / 1000;

    % Weights
    W1 = m1 * g; W2 = m2 * g; W3 = m3 * g;
    Wm2 = m2s * g; Wm3 = m3s * g; Wg = mg * g;
    Wx = payload * g;

    % Update DH with new angles
    DH = [  0       pi/2     130     0;
           120       0        0      0;
            90       0        0      0;
            28       0        0      0];
    DH(:,4) = theta_rad';

    T = eye(4);
    positions = zeros(3, 5);
    positions(:,1) = [0;0;0];

    for i = 1:4
        a = DH(i, 1); alpha = DH(i, 2); d = DH(i, 3); th = DH(i, 4);
        A = [cos(th) -sin(th)*cos(alpha)  sin(th)*sin(alpha) a*cos(th);
             sin(th)  cos(th)*cos(alpha) -cos(th)*sin(alpha) a*sin(th);
             0        sin(alpha)          cos(alpha)         d;
             0        0                   0                  1];
        T = T * A;
        positions(:,i+1) = T(1:3,4);
    end

    %% Torque Calculations
    tau1 = ( ...
        Wx * (L1 + L2 + L3) + ...
        W3 * (L1 + L2 + L3/2) + ...
        Wm3 * (L1 + L2) + ...
        W2 * (L1 + L2/2) + ...
        (Wm2 * L1 + W1 * L1/2) ) / 0.9;

    tau2 = Wx * (L2 + L3) + W3 * (L2 + L3/2) + Wm3 * L2 + W2 * (L2/2);
    tau3 = Wx * L3 + W3 * (L3/2);
    tau4 = Wx * 0.01;

    fprintf('\n--- Static Torques [Nm] (Payload = %.1fg) ---\n', payload * 1000);
    fprintf('Base (θ1):     %.4f Nm\n', tau1);
    fprintf('Shoulder (θ2): %.4f Nm\n', tau2);
    fprintf('Elbow (θ3):    %.4f Nm\n', tau3);
    fprintf('Gripper (θ4):  %.4f Nm\n', tau4);

    ax3D = findobj(gcf, 'Type', 'axes', 'Tag', '');
    if length(ax3D) >= 2
        cla(ax3D(2));
        plot3(ax3D(2), positions(1,:), positions(2,:), positions(3,:), '-o', 'LineWidth', 2);
        xlim(ax3D(2), [-300 300]); ylim(ax3D(2), [-300 300]); zlim(ax3D(2), [0 300]);
        xlabel(ax3D(2), 'X (mm)'); ylabel(ax3D(2), 'Y (mm)'); zlabel(ax3D(2), 'Z (mm)');
        view(ax3D(2), 135, 30);
        grid(ax3D(2), 'on'); axis(ax3D(2), 'equal');

        cla(ax3D(1));
        plot(ax3D(1), positions(1,:), positions(3,:), '-o', 'LineWidth', 2);
        xlim(ax3D(1), [-300 300]); ylim(ax3D(1), [0 300]);
        xlabel(ax3D(1), 'X (mm)'); ylabel(ax3D(1), 'Z (mm)');
        grid(ax3D(1), 'on'); axis(ax3D(1), 'equal');
    end

    set(gcf, 'UserData', struct('sliders', sliders));
end 