# suspension_pro

사용자가 제공한 `HMC_Model_Only.zip`의 MATLAB 차량 모델을 실행하고, 210개 조건의 결과를 `result/`에 저장하는 프로젝트입니다. 모델 방정식과 기본 수치 조건을 보존하며, 실행 진입점과 결과 경로·진행 상태 표시를 정리했습니다.

저장소 준비 시점의 초기 상태는 **0/210 — 실행 환경 및 MathWorks 라이선스 확인 대기**입니다. 초기 manifest와 진행 상태 파일은 실행 계획이며, 계산 완료 결과가 아닙니다. 이후 실제 진행 상황은 [`result/README.md`](result/README.md)와 [`result/progress.json`](result/progress.json)에서 확인합니다. 조건별 상세 상태는 실행 후 생성되는 `result/run_status.csv`에 기록됩니다.

## 실행 조건

| 항목 | 기본값 |
| --- | --- |
| 속도 | 50, 60, 70, 80, 90, 100 km/h |
| 노면 마찰계수 | 0.9, 0.8, 0.7, 0.6, 0.5 |
| 조향 명령 | 0, 0.5, 1, 1.5, 2, 2.5, 3 deg |
| 타이어 마모 FL/FR/RL/RR | `[0 0 0 0]` — 신품 한 패턴 |
| 적분 | 10초, 0.001초 간격, 원본 RK4 및 CDC 샘플·지연 |
| 초기 횡속도 / 요레이트 | 0.1 m/s / 0.35 rad/s |
| CDC 전류 / 스프링 모드 | `[950 950 950 950]` mA / `[2 2 2 2]` |
| 모델 / 보정 | `revised` / 포함된 `calibration_active.json` 적용 |

`6 × 5 × 7 = 210`개 조건이며, 각 조건은 독립적인 초기 상태에서 실행됩니다. 원본에 설명된 다른 마모 패턴까지 추가하면 총 궤적 수가 달라집니다.

## MATLAB에서 한 번에 실행

정식 MATLAB 설치와 사용 가능한 MathWorks 라이선스가 필요합니다. 기본 MATLAB만으로 실행할 수 있으며, MATLAB Runtime만으로는 이 `.m` 소스를 실행할 수 없습니다. 배치 실행에는 별도의 Simulink·최적화·병렬 툴박스나 외부 CarSim 출력이 필요하지 않습니다. 필요한 차량·타이어·댐퍼 데이터는 `data/hmc_data.mat`에 포함되어 있습니다.

MATLAB의 현재 폴더를 저장소로 지정한 뒤 실행합니다.

```matlab
Summary = run_suspension_pro;
```

Linux 셸에서는 다음 한 명령으로 실행하고, 화면 출력과 `result/execution.log`에 로그를 남깁니다.

```bash
bash scripts/run_matlab.sh
```

MATLAB이 PATH에 없다면 설치 폴더를 지정합니다.

```bash
MATLAB_ROOT=/path/to/MATLAB/R2026b bash scripts/run_matlab.sh
```

라이선스 로그인·활성화는 MATLAB에서 완료해야 합니다. 비밀번호나 라이선스 비밀값을 소스나 결과 파일에 기록하지 마세요.

## 가속기와 실행 전 확인

`run_suspension_pro`는 선택된 지원 C++ 컴파일러가 있으면 선택 가속기인 `hmc_mex`를 빌드합니다. 가속기가 있는 경우 기존 `verify_hmc_runtime`의 MATLAB/C++ 구현 동등성 검사에 통과한 뒤에만 사용합니다. 자동 모드에서 가속기가 없거나 검사가 실패하면 순수 MATLAB(`mcode`)으로 실행하며, 그 선택을 로그에 표시합니다. C++ 설정이 필요하면 MATLAB에서 `mex -setup C++`를 실행합니다.

계산 없이 210개 조건을 확인하거나 첫 세 조건만 실행할 수 있습니다.

```matlab
Manifest = run_suspension_pro(struct('dry_run',true));
Summary = run_suspension_pro(struct('case_ids',1:3));
```

순수 MATLAB을 명시하려면 다음과 같이 실행합니다.

```matlab
Summary = run_suspension_pro(struct('options',struct('engine','mcode')));
```

## 결과와 진행 표시

실행 로그는 `[1/210]`, `[2/210]` 형식으로 현재 처리 조건을 표시합니다. 진행 파일은 성공 완료·재사용·실패를 구분하며, 실패 조건을 계산 완료로 세지 않습니다. 일부 조건만 선택하면 로그의 분모는 선택된 조건 수입니다.

| `result/` 파일 | 내용 |
| --- | --- |
| `case_0001_*.mat` 등 | 조건별 실제 계산 결과, 조건 명세 및 입력 서명 |
| `manifest_210.csv` | 전체 조건 목록 |
| `run_status.csv`, `run_status.mat` | 조건별 상태, 소요 시간 및 오류 |
| `README.md`, `progress.json` | 완료 수를 `완료/전체` 형식으로 표시하는 진행 상태 |
| `run_configuration.mat` | 실행 설정 및 재개 확인용 입력 서명 |
| `runtime_verification.mat` | MEX 사용 전 구현 동등성 검사 통과 기록(검사한 경우) |
| `execution.log` | 셸 실행기로 실행했을 때의 MATLAB 콘솔 로그 |

조건별 MAT에는 시간, 34개 상태, 횡속도·요레이트·롤각·롤/피치 각속도·종속도 등 응답과 바퀴별 힘이 저장됩니다. 기본 10초 결과는 10,001개 시간 샘플을 포함합니다. MAT·CSV·PNG·진행 파일·실행 로그는 Git 추적 대상이며, 컴파일된 가속기와 임시 캐시는 제외합니다.

중단 후 같은 명령을 다시 실행하면 입력 서명이 일치하고 검증을 통과한 결과를 재사용합니다. 모델·데이터·보정·설정을 바꾼 경우 기존 결과의 덮어쓰기를 막습니다. 이때는 다른 `output_dir`를 지정하거나, 덮어쓰기가 필요한 범위를 확인한 후 `overwrite=true`를 명시합니다. 컴파일된 MEX나 실행 폴더 경로가 바뀌어도 서명이 달라질 수 있습니다.

일부 조건이 실패하면 나머지 조건은 계속 처리하고 오류를 상태 파일에 남깁니다. 최종적으로 실패가 남으면 통합 실행기는 오류로 종료합니다. 이 모델의 예측 결과나 MATLAB/C++ 동등성 검사 통과를 새로운 CarSim 조건에 대한 정확도 검증으로 해석하지 않습니다.
