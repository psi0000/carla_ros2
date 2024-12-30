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

%--------------------------------------------------------------------------
% EKF 초기화
%   상태벡터 x = [ px, py, vx, vy, qw, qx, qy, qz ]^T
%--------------------------------------------------------------------------
% Constrained EKF 초기화
x_constrained = [0; 0; 0; 0; 1; 0; 0; 0];
P_constrained = eye(8) * 100;

% Unconstrained EKF 초기화
x_unconstrained = [0; 0; 0; 0; 1; 0; 0; 0];
P_unconstrained = eye(8) * 100;

Q = diag([0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001]); % Process noise
R = diag([35^2, 35^2]); % GNSS 측정 노이즈
dt = 0.1; % 시간 간격
num_iterations = 370; % 반복 횟수

%--------------------------------------------------------------------------
% 맵 시각화 초기화
%--------------------------------------------------------------------------
figure;
imshow(mapImage, 'XData', [1, width], 'YData', [1, height]);
hold on;
title('Dynamic Map with Constrained and Unconstrained Positions');
xlabel('X (pixels)');
ylabel('Y (pixels)');

true_values = zeros(num_iterations, 2);         % True (x, y)
noisy_values = zeros(num_iterations, 2);        % Noisy (x, y)
filtered_values_constrained = zeros(num_iterations, 2); % Constrained Filtered
filtered_values_unconstrained = zeros(num_iterations, 2); % Unconstrained Filtered

distance_error_unconstrained = zeros(num_iterations, 1);
distance_error_constrained = zeros(num_iterations, 1);
kalman_gain_constrained = zeros(num_iterations, 1); % Kalman Gain의 평균값 저장 (Constrained)
kalman_gain_unconstrained = zeros(num_iterations, 1); % Kalman Gain의 평균값 저장 (Unconstrained)

%--------------------------------------------------------------------------
% EKF와 센서 데이터 반복 처리
%--------------------------------------------------------------------------
for t = 1:num_iterations
    %--------------------------------------------------------------------------
    % 센서 데이터 가져오기
    %--------------------------------------------------------------------------
    imuMsg = receive(imu_sub, 10); % IMU 데이터 수신
    gnssMsg = receive(gnss_sub, 10); % GNSS 데이터 수신

    % IMU 데이터 파싱
    true_gyro = [imuMsg.angular_velocity.x; ...
                 imuMsg.angular_velocity.y; ...
                 imuMsg.angular_velocity.z];

    % GNSS 데이터 파싱
    true_lat = gnssMsg.latitude;
    true_lon = gnssMsg.longitude;

    % GNSS 데이터를 로컬 좌표로 변환
    earth_radius = 6378137;
    lat_ref = 0.0;
    lon_ref = 0.0;
    x_true = (true_lon - lon_ref) * cosd(lat_ref) * pi/180 * earth_radius;
    y_true = (true_lat - lat_ref) * pi/180 * earth_radius;

    % 노이즈 추가
    gnss_noise_std = 35;
    noisy_x = x_true + randn() * gnss_noise_std;
    noisy_y = y_true + randn() * gnss_noise_std;

    %--------------------------------------------------------------------------
    % Unconstrained EKF 예측 및 갱신
    %--------------------------------------------------------------------------
    % 예측 단계
    x_unconstrained(1:2) = x_unconstrained(1:2) + dt * x_unconstrained(3:4); % 위치 업데이트
    F = eye(8); F(1:2, 3:4) = eye(2) * dt;
    P_unconstrained = F * P_unconstrained * F' + Q * dt^2;

    % 갱신 단계
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        z = [noisy_x; noisy_y];
        H = [eye(2), zeros(2, 6)];
        S = H * P_unconstrained * H' + R;
        K = P_unconstrained * H' / S;
        x_unconstrained = x_unconstrained + K * (z - H * x_unconstrained);
        P_unconstrained = (eye(8) - K * H) * P_unconstrained;
    end

    %--------------------------------------------------------------------------
    % Constrained EKF 예측 및 갱신
    %--------------------------------------------------------------------------
    % 예측 단계
    x_constrained(1:2) = x_constrained(1:2) + dt * x_constrained(3:4); % 위치 업데이트
    P_constrained = F * P_constrained * F' + Q * dt^2;

    % 갱신 단계 (GNSS 데이터와 제약 조건 추가)
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        z = [noisy_x; noisy_y];
        H = [eye(2), zeros(2, 6)];
        S = H * P_constrained * H' + R;
        K = P_constrained * H' / S;
        x_constrained = x_constrained + K * (z - H * x_constrained);
        P_constrained = (eye(8) - K * H) * P_constrained;
    end

    % 도로 방향 제약 (예: 왼쪽 도로는 y ~ 0 근처 유지)
    roadSide = mod(t, 2); % 짝수는 왼쪽, 홀수는 오른쪽
    if roadSide == 0
        D = [0 1 0 0 0 0 0 0];
        d = 0; % 왼쪽 도로
        R_d = 10^2; % 제약 조건 신뢰도
    else
        D = [0 1 0 0 0 0 0 0];
        d = 10; % 오른쪽 도로
        R_d = 10^2;
    end

    % Constrained EKF에 의사 측정 업데이트
    S_pseudo = D * P_constrained * D' + R_d;
    K_pseudo = (P_constrained * D') / S_pseudo;
    x_constrained = x_constrained + K_pseudo * (d - D * x_constrained);
    P_constrained = (eye(8) - K_pseudo * D) * P_constrained;

    %--------------------------------------------------------------------------
    % 데이터 저장
    %--------------------------------------------------------------------------
    true_values(t, :) = [x_true, y_true];
    noisy_values(t, :) = [noisy_x, noisy_y];
    filtered_values_unconstrained(t, :) = [x_unconstrained(1), x_unconstrained(2)];
    filtered_values_constrained(t, :) = [x_constrained(1), x_constrained(2)];
    
    %--------------------------------------------------------------------------
    % 에러값 계산 (True vs Filtered)
    %--------------------------------------------------------------------------
    distance_error_unconstrained(t) = sqrt((x_true - x_unconstrained(1))^2 + (y_true - x_unconstrained(2))^2);
    distance_error_constrained(t) = sqrt((x_true - x_constrained(1))^2 + (y_true - x_constrained(2))^2);
    
    %--------------------------------------------------------------------------
    % Kalman Gain 계산
    %--------------------------------------------------------------------------
    if ~isnan(noisy_x) && ~isnan(noisy_y)
        % Unconstrained Kalman Gain
        H = [eye(2), zeros(2, 6)];
        S_unconstrained = H * P_unconstrained * H' + R;
        K_unconstrained = P_unconstrained * H' / S_unconstrained;
        kalman_gain_unconstrained(t) = mean(abs(K_unconstrained(:))); % Kalman Gain의 평균값 저장

        % Constrained Kalman Gain
        S_constrained = H * P_constrained * H' + R;
        K_constrained = P_constrained * H' / S_constrained;
        kalman_gain_constrained(t) = mean(abs(K_constrained(:))); % Kalman Gain의 평균값 저장
    end

    %--------------------------------------------------------------------------
    % 시각화 (픽셀 좌표 변환)
    %--------------------------------------------------------------------------
    true_x_pix = (x_true - true_center_x) / scaleX + center_x_pix;
    true_y_pix = height - ((y_true - true_center_y) / scaleY + center_y_pix);
    
    noisy_x_pix = (noisy_x - true_center_x) / scaleX + center_x_pix;
    noisy_y_pix = height - ((noisy_y - true_center_y) / scaleY + center_y_pix);
    
    filtered_x_pix_unconstrained = (x_unconstrained(1) - true_center_x) / scaleX + center_x_pix;
    filtered_y_pix_unconstrained = height - ((x_unconstrained(2) - true_center_y) / scaleY + center_y_pix);
    
    filtered_x_pix_constrained = (x_constrained(1) - true_center_x) / scaleX + center_x_pix;
    filtered_y_pix_constrained = height - ((x_constrained(2) - true_center_y) / scaleY + center_y_pix);
    
    %--------------------------------------------------------------------------
    % 시각화 업데이트
    %--------------------------------------------------------------------------
    scatter(true_x_pix, true_y_pix, 100, 'g', 'filled', 'DisplayName', 'True Position');
    scatter(noisy_x_pix, noisy_y_pix, 20, 'r', 'x', 'DisplayName', 'Noisy Position');
    scatter(filtered_x_pix_unconstrained, filtered_y_pix_unconstrained, 50, 'b', 'filled', 'DisplayName', 'Unconstrained Filtered Position');
    scatter(filtered_x_pix_constrained, filtered_y_pix_constrained, 50, 'm', 'filled', 'DisplayName', 'Constrained Filtered Position');
    
    pause(0.1);
end

%--------------------------------------------------------------------------
% 결과 그래프 시각화
%--------------------------------------------------------------------------
figure;

% 에러 비교 (True vs Filtered)
subplot(3, 1, 1);
plot(1:num_iterations, distance_error_unconstrained, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Unconstrained Error');
hold on;
plot(1:num_iterations, distance_error_constrained, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Constrained Error');
xlabel('Time Step');
ylabel('Distance Error (m)');
title('Distance Error Comparison (Constrained vs Unconstrained)');
legend('show', 'Location', 'best');
grid on;

% Kalman Gain 비교
subplot(3, 1, 2);
plot(1:num_iterations, kalman_gain_unconstrained, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Unconstrained Kalman Gain');
hold on;
plot(1:num_iterations, kalman_gain_constrained, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Constrained Kalman Gain');
xlabel('Time Step');
ylabel('Average Kalman Gain');
title('Kalman Gain Comparison (Constrained vs Unconstrained)');
legend('show', 'Location', 'best');
grid on;

% 필터링된 궤적 비교
subplot(3, 1, 3);
plot(filtered_values_unconstrained(:, 1), filtered_values_unconstrained(:, 2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Unconstrained Path');
hold on;
plot(filtered_values_constrained(:, 1), filtered_values_constrained(:, 2), 'm-', 'LineWidth', 1.5, 'DisplayName', 'Constrained Path');
xlabel('X Position');
ylabel('Y Position');
title('Filtered Path Comparison (Constrained vs Unconstrained)');
legend('show', 'Location', 'best');
grid on;

%--------------------------------------------------------------------------
% 대표 결과값 출력
%--------------------------------------------------------------------------
fprintf('Average Distance Error (Unconstrained): %.2f m\n', mean(distance_error_unconstrained));
fprintf('Average Distance Error (Constrained): %.2f m\n', mean(distance_error_constrained));
fprintf('Average Kalman Gain (Unconstrained): %.2f\n', mean(kalman_gain_unconstrained));
fprintf('Average Kalman Gain (Constrained): %.2f\n', mean(kalman_gain_constrained));