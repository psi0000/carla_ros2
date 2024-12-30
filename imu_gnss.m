%--------------------------------------------------------------------------
% 맵 로드 및 초기화
%--------------------------------------------------------------------------
pngFile = '/home/psi/Desktop/map.png'; % 맵 파일 경로
mapImage = imread(pngFile);

[height, width, ~] = size(mapImage); % 맵 크기 확인
center_x_pix = width / 2;
center_y_pix = height / 2;

true_center_x = 0.0; % 실제 중심 X 좌표
true_center_y = 0.0; % 실제 중심 Y 좌표

scaleX = 0.4; % X축 스케일 (픽셀 -> 실제 거리 변환)
scaleY = 0.8; % Y축 스케일

%--------------------------------------------------------------------------
% ROS 노드 및 센서 초기화
%--------------------------------------------------------------------------
node = ros2node('/matlab_node'); % ROS2 노드 초기화
imu_sub = ros2subscriber(node, "/carla/ego_vehicle/imu", "sensor_msgs/Imu");
gnss_sub = ros2subscriber(node, "/carla/ego_vehicle/gnss", "sensor_msgs/NavSatFix");

% EKF 초기화
Q = diag([0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001]); % EKF process noise
R = diag([35^2, 35^2]); % Measurement noise
x = [0; 0; 0; 0; 1; 0; 0; 0]; % [px, py, vx, vy, qw, qx, qy, qz]
P = eye(8) * 100; % Initial covariance matrix
dt = 0.1; % Time step
num_iterations = 800; % 반복 횟수

%--------------------------------------------------------------------------
% 맵 시각화 초기화
%--------------------------------------------------------------------------
figure;
imshow(mapImage, 'XData', [1, width], 'YData', [1, height]);
hold on;
title('Dynamic Map with True and Noisy Positions');
xlabel('X (pixels)');
ylabel('Y (pixels)');


true_values = zeros(num_iterations, 2); % True 값 저장 (x, y)
filtered_values = zeros(num_iterations, 2); % Filtered 값 저장 (x, y)
    

%--------------------------------------------------------------------------
% EKF와 센서 데이터 반복 처리
%--------------------------------------------------------------------------
for t = 1:num_iterations
    t
    %--------------------------------------------------------------------------
    % 센서 데이터 가져오기
    %--------------------------------------------------------------------------
    imuMsg = receive(imu_sub, 10); % IMU 데이터 수신
    gnssMsg = receive(gnss_sub, 10); % GNSS 데이터 수신

    % IMU 데이터 파싱
    true_gyro = [imuMsg.angular_velocity.x; ...
                 imuMsg.angular_velocity.y; ...
                 imuMsg.angular_velocity.z];
    true_quat = [imuMsg.orientation.w, ...
                 imuMsg.orientation.x, ...
                 imuMsg.orientation.y, ...
                 imuMsg.orientation.z];

    % GNSS 데이터 파싱
    true_lat = gnssMsg.latitude;
    true_lon = gnssMsg.longitude;
    true_alt = gnssMsg.altitude;

    % GNSS를 로컬 좌표로 변환
    earth_radius = 6378137;
    lat_ref = 0.0;
    lon_ref = 0.0;
    x_true = (true_lon - lon_ref) * cosd(lat_ref) * pi/180 * earth_radius;
    y_true = (true_lat - lat_ref) * pi/180 * earth_radius;
    z_true = true_alt;

    % 센서 노이즈 추가
    accel_noise_std = 0.1;
    gyro_noise_std = 0.01;
    gnss_noise_std = 35;
    noisy_accel = randn(1, 3) * accel_noise_std;
    noisy_gyro = true_gyro + randn(1, 3) * gyro_noise_std;
    noisy_x = x_true + randn() * gnss_noise_std;
    noisy_y = y_true + randn() * gnss_noise_std;
    noisy_z = z_true + randn() * gnss_noise_std;
    noisy_x_particles = noisy_x + randn(100, 1) * (gnss_noise_std / 2);
    noisy_y_particles = noisy_y + randn(100, 1) * (gnss_noise_std / 2);

    %--------------------------------------------------------------------------
    % EKF 예측 단계
    %--------------------------------------------------------------------------
    imu_w = noisy_gyro';
    qw = x(5); qx = x(6); qy = x(7); qz = x(8);
    dq = 0.5 * [-qx, -qy, -qz; qw, -qz,     qy; qz,  qw, -qx; -qy,  qx,  qw] * imu_w * dt;
    x(5:8) = [qw; qx; qy; qz] / sqrt(qw^2 + qx^2 + qy^2 + qz^2); % 쿼터니언 정규화
    x(1:2) = x(1:2) + dt * x(3:4);
    F = eye(8); F(1:2, 3:4) = eye(2) * dt;
    P = F * P * F' + Q * dt^2;

    %--------------------------------------------------------------------------
    % EKF 갱신 단계
    %--------------------------------------------------------------------------
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        z = [noisy_x; noisy_y];
        H = [eye(2), zeros(2, 6)];
        S = H * P * H' + R;
        K = P * H' / S;
        x = x + K * (z - H * x);
        P = (eye(8) - K * H) * P;
    end

    %--------------------------------------------------------------------------
    % 픽셀 좌표 변환 (시각화용)
    %--------------------------------------------------------------------------
    true_x_pix = (x_true - true_center_x) / scaleX + center_x_pix;
    true_y_pix = height - ((y_true - true_center_y) / scaleY + center_y_pix);
    noisy_x_pix = (noisy_x - true_center_x) / scaleX + center_x_pix;
    noisy_y_pix = height - ((noisy_y - true_center_y) / scaleY + center_y_pix);
    particle_x_pix = (noisy_x_particles - true_center_x) / scaleX + center_x_pix;
    particle_y_pix = height - ((noisy_y_particles - true_center_y) / scaleY + center_y_pix);
    filtered_x_pix = (x(1) - true_center_x) / scaleX + center_x_pix;
    filtered_y_pix = height - ((x(2) - true_center_y) / scaleY + center_y_pix);

    %--------------------------------------------------------------------------
    % 시각화 업데이트
    %--------------------------------------------------------------------------
    scatter(true_x_pix, true_y_pix, 100, 'g', 'filled', 'DisplayName', 'True Position');
    scatter(noisy_x_pix, noisy_y_pix, 50, 'r', 'x', 'DisplayName', 'Noisy Position');
    scatter(filtered_x_pix, filtered_y_pix, 50, 'yellow', 'filled', 'DisplayName', 'Filtered Position');
    %scatter(particle_x_pix, particle_y_pix, 20, 'b', 'filled', 'DisplayName', 'Particles');

    pause(0.1);
    %--------------------------------------------------------------------------
    % True 값과 추정값 비교를 위한 데이터 저장
    %--------------------------------------------------------------------------
    
    
    % True 값 저장
    true_values(t, :) = [x_true, y_true];
    noisy_values(t,:) = [noisy_x,noisy_y];
    % Filtered 값 저장
    filtered_values(t, :) = [x(1), x(2)];


    
   
end
%--------------------------------------------------------------------------
% True 값, Filtered 값, Noisy 값 그래프 시각화
%--------------------------------------------------------------------------
figure;

% X 좌표 비교
subplot(3, 1, 1);
plot(1:num_iterations, true_values(:, 1), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True X');
hold on;
plot(1:num_iterations, noisy_values(:, 1), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Noisy X');
plot(1:num_iterations, filtered_values(:, 1), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered X');
xlabel('Time Step');
ylabel('X Coordinate');
title('True vs Noisy vs Filtered X Coordinate');
legend('show', 'Location', 'best');
grid on;

% Y 좌표 비교
subplot(3, 1, 2);
plot(1:num_iterations, true_values(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True Y');
hold on;
plot(1:num_iterations, noisy_values(:, 2), 'r:', 'LineWidth', 1.5, 'DisplayName', 'Noisy Y');
plot(1:num_iterations, filtered_values(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered Y');
xlabel('Time Step');
ylabel('Y Coordinate');
title('True vs Noisy vs Filtered Y Coordinate');
legend('show', 'Location', 'best');
grid on;

% True 값과 Noisy 값, Filtered 값 간 거리 오차
distance_error_noisy = sqrt((true_values(:, 1) - noisy_values(:, 1)).^2 + ...
                            (true_values(:, 2) - noisy_values(:, 2)).^2);
distance_error_filtered = sqrt((true_values(:, 1) - filtered_values(:, 1)).^2 + ...
                                (true_values(:, 2) - filtered_values(:, 2)).^2);

subplot(3, 1, 3);
plot(1:num_iterations, distance_error_noisy, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Noisy Error');
hold on;
plot(1:num_iterations, distance_error_filtered, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Filtered Error');
xlabel('Time Step');
ylabel('Distance Error (m)');
title('Distance Error: True vs Noisy vs Filtered');
legend('show', 'Location', 'best');
grid on;

hold off;