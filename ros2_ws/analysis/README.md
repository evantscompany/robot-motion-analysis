# analysis

이 디렉터리는 실험 데이터/로그를 분석하기 위한 스크립트들이 모여 있는 곳입니다.

## 주요 스크립트(현재 트리 기준)
- `correction_analysis.py`: correction 관련 분석
- `compare_correction.py`: 보정 결과 비교
- `drift_analysis.py`: 드리프트 분석
- `plot_trajectory.py`: 궤적(trajectory) 플롯
- `live_plot.py`: 실시간 플롯

## 산출물
- `docs/` 아래에 이미지(예: `trajectory_plot.png`, `correction_comparison.png`)가 존재

## 권장 워크플로우(개략)
1. `phases/` 실행을 통해 로봇 데이터를 기록
2. `analysis/` 스크립트로 보정/드리프트/궤적을 분석
3. 결과 이미지를 `docs/`에 정리

