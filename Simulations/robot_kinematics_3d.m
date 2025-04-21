% robot_kinematics_full.m
% Full 4DOF Robot: Kinematic Modelling, FK, IK with Joint Limits + 3D & 2D GUI Sliders

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

%% GUI Setup with Sliders
fig = figure('Name', '4DOF Robot Control Panel', 'Position', [100, 100, 1000, 600]);
slider_vals = zeros(1, DOF);
sliders = gobjects(1, DOF);
labels = {'Base (θ1)', 'Shoulder (θ2)', 'Elbow (θ3)', 'Gripper (θ4)'};
slider_limits = [0 360; 0 180; 0 180; 0 75];

for i = 1:DOF
    idx = DOF - i + 1;  % reverse the slider order for correct top-down mapping
    uicontrol('Style', 'text', 'Position', [50, 540 - (i-1)*100, 100, 20], 'String', labels{idx});
    sliders(idx) = uicontrol('Style', 'slider', 'Min', slider_limits(idx,1), 'Max', slider_limits(idx,2), ...
        'Value', slider_limits(idx,1), 'Position', [160, 540 - (i-1)*100, 400, 20], ...
        'Callback', @(src,~) update_robot());
end

% Axes for 3D and 2D plots
ax3D = axes('Units', 'pixels', 'Position', [600, 300, 350, 250]);
title(ax3D, '3D Robot Arm');
xlabel(ax3D, 'X (mm)'); ylabel(ax3D, 'Y (mm)'); zlabel(ax3D, 'Z (mm)');
grid(ax3D, 'on'); axis(ax3D, 'equal'); view(ax3D, 135, 30);

ax2D = axes('Units', 'pixels', 'Position', [600, 30, 350, 250]);
title(ax2D, '2D Side View (XZ-Plane)');
xlabel(ax2D, 'X (mm)'); ylabel(ax2D, 'Z (mm)');
grid(ax2D, 'on'); axis(ax2D, 'equal');

update_robot();  % initial render

%% Update Function
function update_robot()
    % Access shared variables
    fig_data = get(gcf, 'UserData');
    if isempty(fig_data)
        sliders = findall(gcf, 'Style', 'slider');
    else
        sliders = fig_data.sliders;
    end

    theta_deg = zeros(1, 4);
    for j = 1:4
        theta_deg(j) = get(sliders(j), 'Value');
    end

    theta_rad = deg2rad(theta_deg);
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

    % Clear and re-plot
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
