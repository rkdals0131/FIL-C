# PRD: Ouster OS1-32 LiDAR 포인트클라우드 수직 보간 시스템 (최종)

## 목차
1. [프로젝트 개요](#1-프로젝트-개요)
2. [현황 분석](#2-현황-분석)
3. [기술 사양](#3-기술-사양)
4. [구현 아키텍처](#4-구현-아키텍처)
5. [개발 계획](#5-개발-계획)
6. [검증 및 테스트](#6-검증-및-테스트)

---

## 1. 프로젝트 개요

### 1.1 목표
Ouster OS1-32 LiDAR의 32채널 포인트클라우드를 수직 방향으로 4배 보간하여 128채널의 고밀도 포인트클라우드를 생성

### 1.2 핵심 요구사항
- **입력**: 32×1024 organized 포인트클라우드
- **출력**: 128×1024 organized 포인트클라우드
- **성능**: 실시간 처리 (≥10Hz)
- **정확도**: 기하학적 왜곡 없음, 원본 포인트 위치 보존

### 1.3 성공 기준
1. 보간된 포인트클라우드의 크기와 형태가 정상적으로 유지
2. 시각적 아티팩트 없음
3. 기존 카메라 융합 시스템과 완벽 호환

---

## 2. 현황 분석

### 2.1 현재 구현의 문제점
```
문제: 보간된 포인트클라우드가 매우 작게 나오고, 위아래가 뒤바뀌며, 뒤틀려 있음
```

**근본 원인**:
1. 2D 이미지 보간을 3D 포인트클라우드에 부적절하게 적용
2. 구면 좌표계의 비선형성 무시
3. Ouster 센서의 실제 좌표 변환 공식 미적용
4. 4배 스케일(32→128)이 너무 높아 기하학적 왜곡 심화

### 2.2 기존 접근법 분석
- **현재 구현**: 이미지 보간 → 3D 변환 (부정확)
- **lidar-camera-fusion**: Range Image 변환 → 2D 보간 → 3D 재구성
- **제안 방식**: 구면 좌표 변환 → 정밀 보간 → Ouster 공식 3D 변환

---

## 3. 기술 사양

### 3.1 센서 사양 (Ouster OS1-32)
```yaml
sensor_specs:
  channels: 32
  horizontal_resolution: 1024
  vertical_fov: [-16.611, -0.264]  # degrees
  lidar_origin_to_beam_origin: 15.806  # mm
  data_format: organized_pointcloud  # 32×1024
```

### 3.2 고도각 분포
```cpp
const std::vector<float> beam_altitude_angles_deg = {
    -16.611, -16.084, -15.557, -15.029, -14.502, -13.975,
    -13.447, -12.920, -12.393, -11.865, -11.338, -10.811,
    -10.283, -9.756,  -9.229,  -8.701,  -8.174,  -7.646,
    -7.119,  -6.592,  -6.064,  -5.537,  -5.010,  -4.482,
    -3.955,  -3.428,  -2.900,  -2.373,  -1.846,  -1.318,
    -0.791,  -0.264
};
```

### 3.3 Ouster 공식 좌표 변환
```cpp
// 공식 문서 기반 정확한 변환
θ_encoder = 2π × (1 - measurement_id / 1024)
θ_azimuth = -2π × (beam_azimuth_angles[i] / 360)  // 보통 0
φ = 2π × (beam_altitude_angles[i] / 360)

x = (r - |n|) × cos(θ_encoder + θ_azimuth) × cos(φ) + offset_x
y = (r - |n|) × sin(θ_encoder + θ_azimuth) × cos(φ) + offset_y
z = (r - |n|) × sin(φ) + offset_z

where |n| = 0.015806m (beam origin offset)
```

---

## 4. 구현 아키텍처

### 4.1 데이터 처리 파이프라인

```
┌─────────────────────────────────────────────────────────────┐
│ 1. 입력 처리 (Input Processing)                              │
├─────────────────────────────────────────────────────────────┤
│ • sensor_msgs/PointCloud2 수신 (32×1024)                    │
│ • PCL 변환 및 유효성 검증                                    │
│ • Organized 구조 확인                                       │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. Range Image 생성 (Range Image Creation)                  │
├─────────────────────────────────────────────────────────────┤
│ • XYZ → Range, Intensity 추출                              │
│ • 32×1024 Range Image 구성                                 │
│ • NaN/Invalid 포인트 마킹                                   │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. 보간 수행 (Interpolation)                                │
├─────────────────────────────────────────────────────────────┤
│ • Range 값: 적응적 보간 (불연속성 검출)                      │
│ • Intensity: 선형 보간                                      │
│ • 고도각: 큐빅 스플라인 보간                                 │
│ • 32×1024 → 128×1024 변환                                  │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. 품질 검증 (Quality Validation)                           │
├─────────────────────────────────────────────────────────────┤
│ • 분산 기반 이상치 검출                                      │
│ • 불연속성 경계 처리                                        │
│ • Invalid 포인트 필터링                                      │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 5. 3D 변환 (3D Reconstruction)                              │
├─────────────────────────────────────────────────────────────┤
│ • Ouster 공식 좌표 변환 적용                                │
│ • 128×1024 Organized PointCloud 생성                       │
│ • 메타데이터 추가 (ring, timestamp)                         │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 핵심 클래스 설계

```cpp
// 1. Range Image 관리 클래스
class OusterRangeImage {
private:
    arma::mat range_;       // 거리 값
    arma::mat intensity_;   // 강도 값
    arma::mat valid_mask_;  // 유효성 마스크
    std::vector<float> altitude_rad_;  // 고도각 (라디안)
    
public:
    void fromPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void interpolate(float scale_factor = 4.0);
    pcl::PointCloud<pcl::PointXYZI> toPointCloud();
};

// 2. 보간 엔진
class InterpolationEngine {
private:
    enum Method { LINEAR, CUBIC, ADAPTIVE };
    
public:
    arma::mat interpolateRange(const arma::mat& range, float scale);
    arma::vec interpolateAltitudes(int target_channels);
    void validateInterpolation(arma::mat& data);
};

// 3. 좌표 변환기
class OusterCoordinateTransform {
private:
    const float beam_origin_offset_ = 0.015806f;
    std::vector<float> beam_altitudes_;
    
public:
    pcl::PointXYZI sphericalToCartesian(
        float range, int col, float altitude_rad);
};

// 4. 메인 처리 노드
class PointCloudInterpolationNode : public rclcpp::Node {
private:
    OusterRangeImage range_image_;
    InterpolationEngine interpolator_;
    OusterCoordinateTransform transformer_;
    
public:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processInterpolation();
    void publishResults();
};
```

### 4.3 보간 알고리즘 상세

#### 4.3.1 고도각 보간 (큐빅 스플라인)
```cpp
arma::vec interpolateAltitudes(const arma::vec& original_altitudes, int target_size) {
    arma::vec x_orig = arma::linspace(0, 31, 32);
    arma::vec x_interp = arma::linspace(0, 31, target_size);
    
    arma::vec altitude_interp;
    arma::interp1(x_orig, original_altitudes, x_interp, altitude_interp, "spline");
    
    return altitude_interp;
}
```

#### 4.3.2 Range 적응적 보간
```cpp
float interpolateRangeAdaptive(float r1, float r2, float t) {
    const float discontinuity_threshold = 0.5f;  // 50cm
    
    if (std::abs(r2 - r1) > discontinuity_threshold) {
        // 불연속성: Nearest Neighbor
        return (t < 0.5f) ? r1 : r2;
    } else {
        // 연속성: 큐빅 보간
        // Hermite 보간 사용 (부드러운 전환)
        float t2 = t * t;
        float t3 = t2 * t;
        float h1 = 2*t3 - 3*t2 + 1;
        float h2 = -2*t3 + 3*t2;
        return h1 * r1 + h2 * r2;
    }
}
```

#### 4.3.3 품질 검증
```cpp
void validateInterpolation(arma::mat& range_data) {
    const float variance_threshold = 0.25f;  // 25cm
    
    // 4개 행씩 묶어서 검증 (원본 1 + 보간 3)
    for (int row = 0; row < 128; row += 4) {
        for (int col = 0; col < 1024; ++col) {
            arma::vec local_ranges = range_data.submat(row, col, 
                                                      std::min(row+3, 127), col);
            float variance = arma::var(local_ranges);
            
            if (variance > variance_threshold) {
                // 고분산 영역: 원본 값으로 대체
                float original_value = range_data(row, col);
                for (int i = 1; i <= 3 && row+i < 128; ++i) {
                    range_data(row+i, col) = original_value;
                }
            }
        }
    }
}
```

---

## 5. 개발 계획

### 5.1 개발 단계

#### Phase 1: MVP 구현 (3일) ✅ 완료
- [x] 테스트 노드 생성 (`test_interpolation_node.cpp`)
- [x] 기본 선형 보간 구현
- [x] 입출력 인터페이스 구축
- [x] 시각화 도구 개발 (`visualize_interpolation.py`)
- [x] Launch 파일 및 빌드 스크립트

#### Phase 2: Range Image 통합 (3일)
- [ ] Armadillo 라이브러리 통합
- [ ] Range Image 변환 구현
- [ ] 2D 보간 최적화

#### Phase 3: 정밀 보간 구현 (4일)
- [ ] 큐빅 스플라인 고도각 보간
- [ ] 적응적 Range 보간
- [ ] Ouster 공식 좌표 변환

#### Phase 4: 품질 개선 (3일)
- [ ] 분산 기반 검증
- [ ] 에지 보존 처리
- [ ] 성능 최적화 (OpenMP)

#### Phase 5: 통합 및 테스트 (2일)
- [ ] 기존 시스템 통합
- [ ] 종합 테스트
- [ ] 문서화

### 5.2 구현 우선순위

1. **즉시 구현 (Day 1-2)**
   ```cpp
   // test_interpolation_node.cpp 생성
   // 간단한 선형 보간으로 파이프라인 검증
   ```

2. **핵심 기능 (Day 3-5)**
   ```cpp
   // Range Image 변환
   // Armadillo 기반 고속 보간
   ```

3. **정밀도 개선 (Day 6-10)**
   ```cpp
   // 큐빅 스플라인 고도각
   // Ouster 공식 좌표 변환
   ```

---

## 6. 검증 및 테스트

### 6.1 단위 테스트
```bash
# 좌표 변환 정확도
ros2 run filc test_coordinate_transform

# 보간 알고리즘
ros2 run filc test_interpolation_accuracy

# Range Image 변환
ros2 run filc test_range_image
```

### 6.2 통합 테스트
```bash
# 실시간 처리
ros2 run filc test_interpolation_node

# 성능 측정
ros2 run filc benchmark_interpolation

# 시각화
rviz2 -d src/filc/rviz/interpolation_test.rviz
```

### 6.3 검증 메트릭
- **기하학적 정확도**: RMSE < 1cm @ 10m
- **처리 속도**: > 10Hz
- **메모리 사용**: < 500MB
- **시각적 품질**: 아티팩트 없음

### 6.4 디버깅 도구
```python
# scripts/validate_interpolation.py
- 입출력 차원 확인
- 포인트 분포 분석
- 실시간 통계 출력
```

---

## 부록 A: 즉시 시작 명령어

```bash
# 1. 의존성 설치
sudo apt install libarmadillo-dev

# 2. 빌드
cd /home/user1/ROS2_Workspace/ros2_ws
colcon build --packages-select filc

# 3. 테스트 실행
source install/setup.bash
ros2 run filc test_interpolation_node

# 4. 모니터링
ros2 topic echo /ouster/interpolated_points --no-arr
```

---

## 부록 B: 설정 파일 수정

```yaml
# config/interpolation_config.yaml
interpolation:
  scale_factor: 4.0
  type: "ADAPTIVE"
  
  range_interpolation:
    method: "ADAPTIVE"
    discontinuity_threshold: 0.5
    
  altitude_interpolation:
    method: "CUBIC_SPLINE"
    
  validation:
    enable: true
    variance_threshold: 0.25
    
ouster_transform:
  use_official_formula: true
  beam_origin_offset_m: 0.015806
```

---

이 PRD를 기반으로 체계적인 개발을 진행합니다.

---

## 7. 향후 확장 계획

### 7.1 이미지 데이터 활용
- **Range/Intensity 이미지 기반 Object Detection**
  - Ouster의 구조화된 이미지 데이터 활용
  - YOLO 등 2D detection 모델 적용
  - 3D 포인트클라우드와 2D detection 결과 융합

### 7.2 멀티모달 특징 융합
- **포인트별 특징 확장**
  - 기존: XYZ + Intensity
  - 확장: XYZ + Intensity + Range + RGB
  - 128채널 dense cloud로 정확한 카메라-LiDAR 매핑

### 7.3 고급 응용 분야
1. **Semantic Segmentation**
   - Dense 포인트클라우드 + RGB 정보
   - 픽셀별/포인트별 라벨링

2. **Multi-sensor Fusion**
   - 여러 카메라와의 동시 융합
   - 시간 동기화 및 캘리브레이션

3. **Real-time SLAM**
   - 고밀도 포인트클라우드로 향상된 맵핑
   - RGB 정보로 시각적 특징점 매칭

---

## 8. 현재 진행 상황 (2024.01)

### 완료된 작업 ✅
1. **Phase 1: MVP 구현**
   - `test_interpolation_node`: 기본 선형 보간 ✓
   - `visualize_interpolation.py`: 실시간 모니터링 ✓
   - `benchmark_interpolation.py`: 성능 비교 도구 ✓

2. **Phase 2-3: 다양한 보간 방법 구현**
   - `image_interpolation_node`: Range/Signal 이미지 보간 ✓
   - `spherical_interpolation_node`: 구면 좌표계 보간 (디버깅 필요)
   - `improved_interpolation_node`: 안정적인 XYZ 직접 보간 ✓

3. **문제 해결**
   - QoS 호환성 문제 해결
   - Signal 이미지 토픽 수정
   - 최적 보간 방법 확정: XYZ 직접 보간

### 현재 상태
- **최적 솔루션**: `improved_interpolation_node` 
  - `/ouster/points`만 필요 (이미지 불필요)
  - 안정적인 XYZ 보간
  - 옵션으로 이미지 특징 추가 가능

### 다음 단계 (우선순위)
1. **즉시 (1주)**
   - ✅ 포인트클라우드 보간 최적화 및 안정화
   - ⬜ 파라미터 튜닝 인터페이스
   - ⬜ 실시간 성능 프로파일링

2. **단기 (2-3주)**
   - ⬜ RGB 카메라 융합 인터페이스 설계
   - ⬜ 카메라 캘리브레이션 도구
   - ⬜ 포인트별 RGB 특징 추가

3. **중기 (1-2개월)**
   - ⬜ 2D Object Detection on Range/Signal 이미지
   - ⬜ 3D Bounding Box 투영
   - ⬜ 멀티모달 특징 융합 (XYZ + Intensity + RGB)

4. **장기 (3개월+)**
   - ⬜ Semantic Segmentation
   - ⬜ Real-time SLAM 통합
   - ⬜ Multi-camera 동시 융합