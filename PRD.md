# Product Requirements Document (PRD)
## Fine-grained Interpolation for LiDAR Continuity (filc)

## 1. 현재 개발 상황

### 완료된 작업 ✅
- **핵심 보간 엔진**: `improved_interpolation_node` 구현 완료
  - 32→128 채널 실시간 보간 (>10Hz)
  - XYZ 직접 보간 방식 (안정성 최우선)
  - 적응적 불연속성 처리
  - OpenMP 병렬화

- **인프라 구축**
  - ROS2 패키지 구조 정립
  - Launch 시스템 및 설정 파일
  - 실시간 모니터링 도구
  - 성능 벤치마크 시스템

- **문제 해결**
  - 2D 이미지 보간 → 3D 직접 보간으로 전환
  - QoS 호환성 (best_effort)
  - 파일 구조 간소화

### 진행 중인 작업 🔄
- 성능 프로파일링 및 최적화
- 런타임 파라미터 동적 변경 기능

## 2. 시스템 아키텍처

### 노드 구조
```
┌─────────────────┐
│ Ouster LiDAR    │
│   (OS1-32)      │
└────────┬────────┘
         │ /ouster/points
         │ (32×1024)
         ▼
┌─────────────────────────┐
│ improved_interpolation  │
│        _node            │
│ ┌─────────────────────┐ │
│ │ 1. 포인트 수신      │ │
│ │ 2. 적응적 보간      │ │
│ │ 3. 병렬 처리        │ │
│ └─────────────────────┘ │
└────────┬────────────────┘
         │ /ouster/improved_interpolated_points
         │ (128×1024)
         ▼
┌─────────────────┐
│ 다운스트림 처리 │
│ (SLAM, 검출 등) │
└─────────────────┘
```

### 데이터 흐름
1. **입력**: Organized PointCloud2 (32 rows × 1024 cols)
2. **보간 과정**:
   - 원본 포인트 보존
   - 채널 간 새 포인트 생성
   - 불연속성 감지 (>0.5m)
   - 적응적 보간 방법 선택
3. **출력**: Organized PointCloud2 (128 rows × 1024 cols)

### 핵심 알고리즘
```cpp
// 의사 코드
for each column:
    for each channel gap:
        if (discontinuity > threshold):
            use_nearest_neighbor()
        else:
            use_linear_interpolation()
```

## 3. 기술 스택

### 의존성
- **필수**: ROS2, PCL, Eigen3, OpenMP
- **선택**: OpenCV (향후 이미지 특징 통합용)

### 성능 지표
- **처리 시간**: ~50ms @ 4x scale
- **메모리 사용**: <500MB
- **CPU 활용**: 멀티코어 병렬화

## 4. 향후 개발 계획

### Phase 1: 최적화 (1-2주)
- [ ] SIMD 명령어 활용
- [ ] 메모리 접근 패턴 최적화
- [ ] 동적 파라미터 재설정

### Phase 2: 카메라 융합 (3-4주)
- [ ] RGB 카메라 동기화 모듈
- [ ] 캘리브레이션 도구
- [ ] 포인트별 색상 매핑

### Phase 3: 고급 기능 (1-2개월)
- [ ] Range/Intensity 이미지 생성
- [ ] 2D 객체 검출 통합
- [ ] 멀티모달 특징 융합

### Phase 4: 확장성 (2-3개월)
- [ ] 다른 LiDAR 모델 지원
- [ ] 동적 해상도 조정
- [ ] GPU 가속 옵션

## 5. 설계 원칙

1. **안정성 우선**: 정확도 > 속도
2. **모듈화**: 각 기능을 독립적으로 개발/테스트
3. **실시간성**: 센서 주파수(10Hz) 이상 유지
4. **확장성**: 새로운 센서/알고리즘 쉽게 추가

## 6. 리스크 및 대응

### 기술적 리스크
- **메모리 대역폭**: 병렬화로 인한 캐시 미스
  - 대응: 타일 기반 처리 고려

- **레이턴시**: 다운스트림 처리 지연
  - 대응: 파이프라인 최적화

### 운영 리스크
- **센서 변경**: OS1-32 외 모델 요구
  - 대응: 설정 파일 기반 확장

## 7. 개발 로드맵

```
2024 Q1: 코어 보간 엔진 ✅
2024 Q2: 성능 최적화 및 카메라 융합 🔄
2024 Q3: 고급 기능 및 확장성
2024 Q4: 프로덕션 준비 및 배포
```

## 8. 참고 자료

- [Ouster 공식 문서](https://static.ouster.dev/sensor-docs/)
- [ROS2 포인트클라우드 처리](https://docs.ros.org/en/humble/Tutorials/Intermediate/PointCloud2-Tutorials.html)
- [PCL 라이브러리](https://pointclouds.org/)