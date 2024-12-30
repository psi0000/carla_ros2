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
road_segmentation_sub = ros2subscriber(node, "/carla/ego_vehicle/road_pointcloud", "sensor_msgs/PointCloud2");
sidewalk_segmentation_sub = ros2subscriber(node, "/carla/ego_vehicle/sidewalk_pointcloud", "sensor_msgs/PointCloud2");

%--------------------------------------------------------------------------
% EKF 초기화
%   상태벡터 x = [ px, py, vx, vy, qw, qx, qy, qz ]^T
%--------------------------------------------------------------------------
Q = diag([0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001]); % Process noise
R = diag([5^2, 5^2]); % GNSS 측정 노이즈
x = [0; 0; 0; 0; 1; 0; 0; 0];     % 초기 상태 (위치 0, 속도 0, quaternion=[1,0,0,0])
P = eye(8) * 100;                % 초기 공분산
dt = 0.1;                        % 시간 간격
num_iterations = 370;            % 반복 횟수

%--------------------------------------------------------------------------
% 맵 시각화 초기화
%--------------------------------------------------------------------------
figure;
imshow(mapImage, 'XData', [1, width], 'YData', [1, height]);
hold on;
title('Dynamic Map with True and Noisy Positions');
xlabel('X (pixels)');
ylabel('Y (pixels)');

true_values = zeros(num_iterations, 2);    % True (x, y)
noisy_values = zeros(num_iterations, 2);   % Noisy (x, y)
filtered_values = zeros(num_iterations, 2);% Filtered (x, y)

%--------------------------------------------------------------------------
% EKF와 센서 데이터 반복 처리
%--------------------------------------------------------------------------
for t = 1:num_iterations
    fprintf("Time step: %d\n", t);
    
    %--------------------------------------------------------------------------
    % (1) 센서 데이터 가져오기
    %--------------------------------------------------------------------------
    imuMsg = receive(imu_sub, 10);  % IMU 데이터 수신
    gnssMsg = receive(gnss_sub, 10);% GNSS 데이터 수신
    road_msg = receive(road_segmentation_sub, 10);    % 도로 점군
    sidewalk_msg = receive(sidewalk_segmentation_sub, 10); % 인도 점군

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

    % (위/경도 -> 로컬 좌표 변환) 단순 예시
    earth_radius = 6378137;
    lat_ref = 0.0;
    lon_ref = 0.0;
    x_true = (true_lon - lon_ref) * cosd(lat_ref) * pi/180 * earth_radius;
    y_true = (true_lat - lat_ref) * pi/180 * earth_radius;
    z_true = true_alt;

    %--------------------------------------------------------------------------
    % (2) 센서 노이즈 추가 (임의 예시)
    %--------------------------------------------------------------------------
    accel_noise_std = 0.1;
    gyro_noise_std = 0.01;
    gnss_noise_std = 30;

    noisy_gyro = true_gyro + randn(3,1) * gyro_noise_std;      % gyro
    noisy_x = x_true + randn() * gnss_noise_std;               % GNSS x
    noisy_y = y_true + randn() * gnss_noise_std;               % GNSS y
    noisy_z = z_true + randn() * gnss_noise_std;               % GNSS z
    noisy_accel = randn(1, 3) * accel_noise_std;               % (실제로는 가속도도 쓸 수 있음)

    road_points = rosReadXYZ(road_msg);      % 도로 점군 데이터
    sidewalk_points = rosReadXYZ(sidewalk_msg); % 인도 점군 데이터

    % 도로/인도 평균 y값 계산 (예시)
    mean_road_y = mean(road_points(:, 2));       
    mean_sidewalk_y = mean(sidewalk_points(:, 2));

    %--------------------------------------------------------------------------
    % (3) EKF 예측 단계
    %--------------------------------------------------------------------------
    imu_w = noisy_gyro;  % (3x1)
    qw = x(5); qx = x(6); qy = x(7); qz = x(8);

    % 쿼터니언 미분 근사: dq = 0.5 * Omega(q) * w
    dq = 0.5 * [-qx, -qy, -qz;
                 qw, -qz,  qy;
                 qz,  qw, -qx;
                -qy,  qx,  qw] * imu_w * dt;

    % 쿼터니언 업데이트
    x(5:8) = [qw; qx; qy; qz] + dq;
    % 노말라이즈
    norm_q = sqrt(x(5)^2 + x(6)^2 + x(7)^2 + x(8)^2);
    x(5:8) = x(5:8)/norm_q;

    % 위치/속도 기본 모델 (단순)
    x(1:2) = x(1:2) + x(3:4)*dt;   % px, py 업데이트

    % 선형화 행렬 F
    F = eye(8);
    F(1,3) = dt; 
    F(2,4) = dt;

    % 공분산 예측
    P = F * P * F' + Q * dt^2; 

    %--------------------------------------------------------------------------
    % (4) GNSS 측정 업데이트 (z = [noisy_x; noisy_y])
    %--------------------------------------------------------------------------
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        z = [noisy_x; noisy_y];
        H = [eye(2), zeros(2,6)];  % (2x8)
        S = H * P * H' + R;        % (2x2)
        K = P * H' / S;            % (8x2)
        x = x + K * (z - H*x);     % 상태 갱신
        P = (eye(8) - K*H)*P;      % 공분산 갱신
    end

    %--------------------------------------------------------------------------
    % (5) 도로/인도 pseudo-measurement 업데이트 (예시)
    %--------------------------------------------------------------------------
    % roadSide = mod(t, 2); % 0 or 1, 여기서는 데모용
    roadSide = 0;  % 임의 지정

    if roadSide == 0
        % ----- Left side : y ~ mean_sidewalk_y -----
        D = [0 1 0 0 0 0 0 0];  % (1x8) -> y
        d = [mean_sidewalk_y];  % 인도 y 평균
        R_d = 10^2;             % pseudo‐measurement 노이즈

    else
        % ----- Right side : x ~ mean_sidewalk_x (가정) -----
        % (실제로는 도로 y값 or 다른 제약을 줄 수도 있음)
        D = [1 0 0 0 0 0 0 0];  
        d = [ mean_sidewalk_x  ]; % 여기는 실제 로직에 맞춰 수정
        R_d = 15^2;
    end

    % 의사측정 업데이트
    S_pseudo = D * P * D' + R_d;    
    K_pseudo = (P * D') / S_pseudo; 
    x = x + K_pseudo * (d - D*x);   
    P = (eye(8) - K_pseudo * D) * P;

    %--------------------------------------------------------------------------
    % (6) 시각화를 위한 픽셀 좌표 변환
    %--------------------------------------------------------------------------
    true_x_pix = (x_true - true_center_x) / scaleX + center_x_pix;
    true_y_pix = height - ((y_true - true_center_y) / scaleY + center_y_pix);

    noisy_x_pix = (noisy_x - true_center_x) / scaleX + center_x_pix;
    noisy_y_pix = height - ((noisy_y - true_center_y) / scaleY + center_y_pix);

    filtered_x_pix = (x(1) - true_center_x) / scaleX + center_x_pix;
    filtered_y_pix = height - ((x(2) - true_center_y) / scaleY + center_y_pix);

    %--------------------------------------------------------------------------
    % (7) 시각화 업데이트 (Scatter)
    %--------------------------------------------------------------------------
    scatter(true_x_pix, true_y_pix, 100, 'g', 'filled', 'DisplayName', 'True Position');
    scatter(noisy_x_pix, noisy_y_pix, 50, 'r', 'x', 'DisplayName', 'Noisy Position');
    scatter(filtered_x_pix, filtered_y_pix, 50, 'y', 'filled', 'DisplayName', 'Filtered Position');
    
    pause(0.1);

    %--------------------------------------------------------------------------
    % (8) True / Noisy / Filtered 값 기록
    %--------------------------------------------------------------------------
    true_values(t, :)    = [x_true, y_true];
    noisy_values(t, :)   = [noisy_x, noisy_y];
    filtered_values(t, :)= [x(1),   x(2)];
end

%--------------------------------------------------------------------------
% (9) 결과 그래프 시각화 (X, Y 비교 및 거리오차)
%--------------------------------------------------------------------------
figure;

% X 좌표 비교
subplot(3, 1, 1);
plot(1:num_iterations, true_values(:, 1), 'g-', 'LineWidth', 1.5); hold on;
plot(1:num_iterations, noisy_values(:, 1), 'r:', 'LineWidth', 1.5);
plot(1:num_iterations, filtered_values(:, 1), 'b--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('X Coordinate');
title('True vs Noisy vs Filtered X');
legend('True X','Noisy X','Filtered X','Location','best'); grid on;

% Y 좌표 비교
subplot(3, 1, 2);
plot(1:num_iterations, true_values(:, 2), 'g-', 'LineWidth', 1.5); hold on;
plot(1:num_iterations, noisy_values(:, 2), 'r:', 'LineWidth', 1.5);
plot(1:num_iterations, filtered_values(:, 2), 'b--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Y Coordinate');
title('True vs Noisy vs Filtered Y');
legend('True Y','Noisy Y','Filtered Y','Location','best'); grid on;

% True 값과 Noisy 값, Filtered 값 간 거리 오차
distance_error_noisy = sqrt((true_values(:,1) - noisy_values(:,1)).^2 + ...
                            (true_values(:,2) - noisy_values(:,2)).^2);
distance_error_filtered = sqrt((true_values(:,1) - filtered_values(:,1)).^2 + ...
                               (true_values(:,2) - filtered_values(:,2)).^2);

subplot(3, 1, 3);
plot(1:num_iterations, distance_error_noisy, 'r:', 'LineWidth', 1.5); hold on;
plot(1:num_iterations, distance_error_filtered, 'b--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Distance Error (m)');
title('Distance Error: True vs Noisy vs Filtered');
legend('Noisy Error','Filtered Error','Location','best');
grid on;

%--------------------------------------------------------------------------
% (10) 오차 분석 (X, Y 각각 / 2D 거리)
%--------------------------------------------------------------------------
x_error_noisy      = true_values(:,1) - noisy_values(:,1);
x_error_filtered   = true_values(:,1) - filtered_values(:,1);

y_error_noisy      = true_values(:,2) - noisy_values(:,2);
y_error_filtered   = true_values(:,2) - filtered_values(:,2);

% 평균 오차
mean_x_err_noisy      = mean(x_error_noisy);
mean_x_err_filtered   = mean(x_error_filtered);
mean_y_err_noisy      = mean(y_error_noisy);
mean_y_err_filtered   = mean(y_error_filtered);

% RMSE
rmse_x_noisy       = sqrt(mean(x_error_noisy.^2));
rmse_x_filtered    = sqrt(mean(x_error_filtered.^2));
rmse_y_noisy       = sqrt(mean(y_error_noisy.^2));
rmse_y_filtered    = sqrt(mean(y_error_filtered.^2));

% 2D 오차 평균 및 RMSE
mean_dist_err_noisy    = mean(distance_error_noisy);
mean_dist_err_filtered = mean(distance_error_filtered);
rmse_dist_noisy        = sqrt(mean(distance_error_noisy.^2));
rmse_dist_filtered     = sqrt(mean(distance_error_filtered.^2));

disp('===== X Error Analysis =====')
fprintf('Noisy   : mean=%.3f, RMSE=%.3f\n', mean_x_err_noisy, rmse_x_noisy);
fprintf('Filtered: mean=%.3f, RMSE=%.3f\n', mean_x_err_filtered, rmse_x_filtered);

disp(' ')
disp('===== Y Error Analysis =====')
fprintf('Noisy   : mean=%.3f, RMSE=%.3f\n', mean_y_err_noisy, rmse_y_noisy);
fprintf('Filtered: mean=%.3f, RMSE=%.3f\n', mean_y_err_filtered, rmse_y_filtered);

disp(' ')
disp('===== 2D Distance Error Analysis =====')
fprintf('Noisy   : mean=%.3f, RMSE=%.3f\n', mean_dist_err_noisy, rmse_dist_noisy);
fprintf('Filtered: mean=%.3f, RMSE=%.3f\n', mean_dist_err_filtered, rmse_dist_filtered);

%--------------------------------------------------------------------------
% (11) 추가 오차 그래프 (옵션)
%--------------------------------------------------------------------------
figure;
subplot(2,1,1);
plot(1:num_iterations, x_error_noisy, 'r:',  'DisplayName','X Error Noisy');  hold on;
plot(1:num_iterations, x_error_filtered, 'b--','DisplayName','X Error Filtered');
xlabel('Time Step'); ylabel('X Error (m)');
legend('Location','best'); grid on; title('X Error over Time');

subplot(2,1,2);
plot(1:num_iterations, y_error_noisy, 'r:',  'DisplayName','Y Error Noisy');  hold on;
plot(1:num_iterations, y_error_filtered, 'b--','DisplayName','Y Error Filtered');
xlabel('Time Step'); ylabel('Y Error (m)');
legend('Location','best'); grid on; title('Y Error over Time');
