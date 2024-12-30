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

% EKF 초기화
Q = diag([0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001]); % EKF process noise
R = diag([5^2, 5^2]); % GNSS Measurement noise
R_pseudo = 10^2; % Pseudo Measurement noise
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
title('Dynamic Map with True and Constrained Positions');
xlabel('X (pixels)');
ylabel('Y (pixels)');

true_values = zeros(num_iterations, 2); % True 값 저장 (x, y)
filtered_values = zeros(num_iterations, 2); % Filtered 값 저장 (x, y)

%--------------------------------------------------------------------------
% EKF와 센서 데이터 반복 처리
%--------------------------------------------------------------------------
for t = 1:num_iterations
    %--------------------------------------------------------------------------
    % 센서 데이터 가져오기
    %--------------------------------------------------------------------------
    imuMsg = receive(imu_sub, 10); % IMU 데이터 수신
    gnssMsg = receive(gnss_sub, 10); % GNSS 데이터 수신
    road_msg = receive(road_segmentation_sub, 10); % 도로 Segmentation 데이터
    sidewalk_msg = receive(sidewalk_segmentation_sub, 10); % 도로 Segmentation 데이터

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
    gnss_noise_std = 25;
    noisy_x = x_true + randn() * gnss_noise_std
    noisy_y = y_true + randn() * gnss_noise_std;
    
    %--------------------------------------------------------------------------
    % 인도와 도로의 관계를 기반으로 Pseudo 측정값 계산
    %--------------------------------------------------------------------------
    % 도로와 인도의 평균 y 값을 계산
    road_points = rosReadXYZ(road_msg); % 도로 점군 데이터
    sidewalk_points = rosReadXYZ(sidewalk_msg); % 인도 점군 데이터
    
    mean_road_y = mean(road_points(:, 2)); % 도로의 평균 y 값
    mean_sidewalk_y = mean(sidewalk_points(:, 2)); % 인도의 평균 y 값
    
    % Heading 값 계산
    rpy = quat2eul([x(5), x(6), x(7), x(8)], 'ZYX');
    theta = rpy(1); % Heading 값 (0도 동쪽 기준)
    
    if abs(cos(theta)) > abs(sin(theta)) % 가로 도로
        if theta >= -pi/4 && theta <= pi/4 % 동쪽 방향 (0도 부근)
            if mean_road_y > mean_sidewalk_y
                % 도로의 y 값이 보도의 y 값보다 큰 경우
                disp('도로가 보도 위쪽에 있습니다. y 값을 낮춥니다.');
                h1 = x(2)-sin(theta)*(x(2) - mean_sidewalk_y);
                R_pseudo = 20^2; % 높은 신뢰도
            else
                % 도로의 y 값이 보도의 y 값보다 작은 경우
                disp('도로가 보도 아래쪽에 있습니다. y 값을 높입니다.');
                h1 = x(2) - mean_sidewalk_y;
                R_pseudo = 1^2; % 높은 신뢰도
            end
        elseif theta >= 3*pi/4 || theta <= -3*pi/4 % 서쪽 방향 (180도 부근)
            if mean_road_y > mean_sidewalk_y
                % 도로의 y 값이 보도의 y 값보다 큰 경우
                disp('서쪽: 도로가 보도 위쪽에 있습니다. y 값을 낮춥니다.');
                h1 = x(2) - mean_sidewalk_y;
                R_pseudo = 1^2; % 높은 신뢰도
            else
                % 도로의 y 값이 보도의 y 값보다 작은 경우
                disp('서쪽: 도로가 보도 아래쪽에 있습니다. y 값을 높입니다.');
                h1 = x(2) - mean_sidewalk_y;
                R_pseudo = 1^2; % 높은 신뢰도
            end
        else
            disp('도로가 세로 방향입니다. 제한 없음.');
            h1 = 0; % 세로 도로는 제한하지 않음
            R_pseudo = 10^6; % 신뢰도 최소화
        end
    else
        disp('도로가 세로 방향입니다. 제한 없음.');
        h1 = 0; % 세로 도로는 제한하지 않음
        R_pseudo = 10^6; % 신뢰도 최소화
    end
    h2 = x(3) * cos(theta) + x(4) * sin(theta); % Heading 방향 속도 제약

    %--------------------------------------------------------------------------
    % EKF 갱신 단계 (통합 갱신)
    %--------------------------------------------------------------------------
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        % 측정값 및 Pseudo 측정값 결합
        z_combined = [noisy_x; noisy_y; h1; h2];
        H_combined = [eye(2), zeros(2, 6);  % GNSS 측정
              0, 1-sin(theta), 0, 0, 0, 0, 0, 0;
              0, 0, cos(theta), sin(theta), 0, 0, 0, 0]; % Pseudo 측정
        
        R_combined = blkdiag(R, R_pseudo, 0.1^2); % 블록 대각 행렬
    
        % 통합 공분산 및 칼만 이득 계산
        S_combined = H_combined * P * H_combined' + R_combined;
        K_combined = P * H_combined' / S_combined;
    
        % 상태 갱신
        x = x + K_combined * (z_combined - H_combined * x);
        P = (eye(8) - K_combined * H_combined) * P; % 공분산 갱신
        
    end


    %--------------------------------------------------------------------------
    % True 값과 추정값 비교를 위한 데이터 저장
    %--------------------------------------------------------------------------
    true_values(t, :) = [x_true, y_true];
    filtered_values(t, :) = [x(1), x(2)];

    %--------------------------------------------------------------------------
    % 시각화 업데이트
    %--------------------------------------------------------------------------
    true_x_pix = (x_true - true_center_x) / scaleX + center_x_pix;
    true_y_pix = height - ((y_true - true_center_y) / scaleY + center_y_pix);
    filtered_x_pix = (x(1) - true_center_x) / scaleX + center_x_pix;
    filtered_y_pix = height - ((x(2) - true_center_y) / scaleY + center_y_pix);

    scatter(true_x_pix, true_y_pix, 100, 'g', 'filled', 'DisplayName', 'True Position');
    scatter(filtered_x_pix, filtered_y_pix, 50, 'r', 'filled', 'DisplayName', 'Filtered Position');
    pause(0.1);
end

%--------------------------------------------------------------------------
% True 값, Filtered 값 그래프 시각화
%--------------------------------------------------------------------------
figure;

subplot(2, 1, 1);
plot(1:num_iterations, true_values(:, 1), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True X');
hold on;
plot(1:num_iterations, filtered_values(:, 1), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Filtered X');
xlabel('Time Step');
ylabel('X Coordinate');
title('True vs Filtered X Coordinate');
legend('show', 'Location', 'best');
grid on;

subplot(2, 1, 2);
plot(1:num_iterations, true_values(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True Y');
hold on;
plot(1:num_iterations, filtered_values(:, 2), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Filtered Y');
xlabel('Time Step');
ylabel('Y Coordinate');
title('True vs Filtered Y Coordinate');
legend('show', 'Location', 'best');
grid on;
