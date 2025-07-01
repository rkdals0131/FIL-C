# Fine-grained Interpolation for LiDAR Continuity

## 1. 빌드

```bash
# 빌드 스크립트 실행
cd /home/user1/ROS2_Workspace/ros2_ws/src/filc/scripts
./build_and_test.sh

# 또는 수동 빌드
cd /home/user1/ROS2_Workspace/ros2_ws
colcon build --packages-select filc
source install/setup.bash
```

## 2. 실행 방법

### 방법 1: Launch 파일 사용 (권장)
```bash
ros2 launch filc test_interpolation.launch.py
```

옵션 설정:
```bash
ros2 launch filc test_interpolation.launch.py \
  input_topic:=/ouster/points \
  output_topic:=/ouster/interpolated_points \
  scale_factor:=4.0
```

### 방법 2: 개별 노드 실행
```bash
# Terminal 1: 보간 노드
ros2 run filc test_interpolation_node

# Terminal 2: 시각화
ros2 run filc visualize_interpolation.py
```

## 3. 확인

### 토픽 확인
```bash
# 토픽 리스트
ros2 topic list

# 출력 확인
ros2 topic echo /ouster/interpolated_points --no-arr --once

# 주파수 확인
ros2 topic hz /ouster/interpolated_points
```

### RViz2 시각화
```bash
rviz2

# PointCloud2 디스플레이 추가
# Topic: /ouster/interpolated_points
# Fixed Frame: os_sensor 또는 ouster_lidar
```

## 4. 파라미터 조정

### 런타임 파라미터 변경
```bash
# 스케일 팩터 변경 (예: 2배 보간)
ros2 param set /test_interpolation_node scale_factor 2.0
```

### 사용 가능한 파라미터
- `input_topic`: 입력 포인트클라우드 토픽 (기본: /ouster/points)
- `output_topic`: 출력 포인트클라우드 토픽 (기본: /ouster/interpolated_points)
- `scale_factor`: 보간 배율 (기본: 4.0, 32→128 채널)

## 5. 예상 결과

### 정상 작동 시:
```
[INFO] Test Interpolation Node Started
  Input: /ouster/points
  Output: /ouster/interpolated_points
  Scale Factor: 4.0

[INFO] === Interpolation Statistics ===
  Frames processed: 50
  Average time: 45.32 ms
  Average FPS: 22.1
  Scale factor: 4.0x (32->128 channels)
```

### 시각화 출력:
```
============================================================
Interpolation Statistics - 14:35:22
============================================================
Original PointCloud:
  Dimensions: 32x1024
  Total points: 32,768
  Data size: 0.50 MB
Interpolated PointCloud:
  Dimensions: 128x1024
  Total points: 131,072
  Data size: 2.00 MB
Comparison:
  Height ratio: 4.00x
  Points ratio: 4.00x
  Data size ratio: 4.00x
Frames processed: 120
============================================================
```

## 6. 문제 해결

### 데이터가 없을 때
```bash
# Ouster 드라이버 실행 확인
ros2 node list | grep ouster

# 토픽 데이터 확인
ros2 topic echo /ouster/points --once
```

### 성능 이슈
- `scale_factor`를 낮춰보세요 (예: 2.0)
- CPU 사용률 확인: `htop`

### 보간 품질
- 현재는 간단한 선형 보간 사용
- 향후 구면 좌표계 기반 정밀 보간으로 업그레이드 예정

## 7. 다음 단계

1. **Range Image 기반 보간 구현**
   - Armadillo 라이브러리 통합
   - 2D 보간 최적화

2. **구면 좌표계 정밀 보간**
   - 큐빅 스플라인 고도각 보간
   - Ouster 공식 좌표 변환

3. **품질 개선**
   - 분산 기반 검증
   - 에지 보존 처리