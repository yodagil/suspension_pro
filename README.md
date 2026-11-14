# suspension_pro

사용자가 제공한 `HMC_Model_Only.zip`의 MATLAB 차량 모델을 실행하고, **210개 운전 조건 × 5개 타이어 마모 패턴 = 총 1,050개 궤적**의 결과를 `result/`에 저장하는 프로젝트입니다. 모델 방정식과 기본 수치 조건을 보존하며, 원본에서 설명한 다섯 마모 패턴을 모두 실행하도록 기본 구성을 정리했습니다.

원본의 바퀴별 횡력 진단값 `Fy`가 차체 합력으로 덮어써지는 오류는 합력 변수 이름만 바꾸어 수정했습니다. 세 표본 상태에서 수정 전후 34개 상태 미분값이 비트 단위로 동일함을 확인했습니다. 변경 이력과 원본 파일 해시는 `source_provenance.json`에 기록합니다.

GNU Octave 10.3.0과 제공된 C++ 가속기로 계산합니다. 전체 및 마모 패턴별 실제 진행 상황은 [`result/README.md`](result/README.md)와 [`result/progress.json`](result/progress.json)에서 확인합니다. 조건별 상태와 오류는 `result/run_status.csv`에 기록됩니다.

전체 실행 전에 CarSim 3개 조건을 기존·수정 모델로 각각 실행했습니다. **제공 CSV의 18개 신호 행과 162개 수치를 모두 재현**했으며, 전체 구간 RMSE의 최대 차이는 `5.09e-15`입니다. [비교 결과·CSV·그래프](result/validation/README.md)에 실제 계산 결과와 검증 범위를 저장했습니다.

## 실행 조건

| 항목 | 기본값 |
| --- | --- |
| 속도 | 50, 60, 70, 80, 90, 100 km/h |
| 노면 마찰계수 | 0.9, 0.8, 0.7, 0.6, 0.5 |
| 조향 명령 | 0, 0.5, 1, 1.5, 2, 2.5, 3 deg |
| 타이어 마모 FL/FR/RL/RR | 아래 원본의 다섯 패턴 전체 |
| 적분 | 10초, 0.001초 간격, 원본 RK4 및 CDC 샘플·지연 |
| 초기 횡속도 / 요레이트 | 0.1 m/s / 0.35 rad/s |
| CDC 전류 / 스프링 모드 | `[950 950 950 950]` mA / `[2 2 2 2]` |
| 모델 / 보정 | `revised` / 포함된 `calibration_active.json` 적용 |

패턴별로 `6 × 5 × 7 = 210`개 운전 조건을 계산합니다. 다음 다섯 마모 패턴을 포함하므로 총 `210 × 5 = 1050`개 궤적이며, 각 궤적은 독립적인 초기 상태에서 실행됩니다. 마모 값 `0`, `25`, `50`은 각각 신품, 25K, 50K를 뜻합니다.

| WearCaseID | FL | FR | RL | RR | 전체 JobID |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | 0 | 0 | 0 | 0 | 1–210 |
| 2 | 25 | 25 | 0 | 0 | 211–420 |
| 3 | 50 | 50 | 25 | 25 | 421–630 |
| 4 | 0 | 0 | 25 | 25 | 631–840 |
| 5 | 25 | 25 | 50 | 50 | 841–1050 |

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

계산 없이 전체 1,050개 궤적의 조건을 확인하거나 전체 JobID 중 첫 세 조건만 실행할 수 있습니다.

```matlab
Manifest = run_suspension_pro(struct('dry_run',true));
Summary = run_suspension_pro(struct('case_ids',1:3));
```

순수 MATLAB을 명시하려면 다음과 같이 실행합니다.

```matlab
Summary = run_suspension_pro(struct('options',struct('engine','mcode')));
```

## GNU Octave 호환 실행

실제 검증 환경은 GNU Octave 10.3.0, Tablicious 0.4.7, GCC 16.2.0입니다. Octave는 정식 MATLAB과 다른 실행 환경입니다. `table`에는 Tablicious를 사용하고, CSV와 진행 상태 저장은 입출력 호환 계층을 사용합니다. 차량 모델의 적분식은 동일합니다.

Octave에서는 통합 실행기가 MATLAB 전용 컴파일러 설정 API를 호출하지 않습니다. `mkoctfile --mex`로 미리 빌드한 `hmc_mex`가 있으면 기존 m-code/native 구현 동등성 검사를 통과한 뒤 사용하며, 없으면 원본 m-code로 실행합니다. 검사를 수행한 실행 환경과 버전은 `runtime_verification.mat`에 기록합니다. 전체 실행 전 짧은 대표 계산, 가속기 동등성, 결과 저장을 실제로 검증해야 합니다.

Octave와 C++ 개발 도구를 설치하고 Tablicious v0.4.7 소스를 준비한 환경에서 실행합니다.

```bash
TABLICIOUS_ROOT=/path/to/octave-tablicious bash scripts/run_octave.sh
```

이 실행기는 Octave용 MEX 빌드와 수치 동등성 검사를 수행하고 `result/execution.log`에 실행 기록을 남깁니다. 자세한 호환 처리와 의존성 버전은 [`compat/octave/README.md`](compat/octave/README.md)를 참고하세요.

## 결과와 진행 표시

실행 로그와 결과 목록은 전체 `1/1050`, `2/1050` 형식의 JobID와 각 마모 패턴 안의 `1/210` 형식 ConditionID를 함께 표시합니다. 진행 파일은 전체 성공 수와 마모 패턴별 성공 수를 별도로 집계하며, 성공 완료·재사용·실패를 구분합니다. 실패 조건은 성공 완료로 세지 않습니다. 일부 JobID만 선택한 실행에서도 전체 계획과 선택하지 않은 조건을 구분해 기록합니다.

| `result/` 파일 | 내용 |
| --- | --- |
| `case_0001_*.mat` 등 | 조건별 실제 계산 결과, 조건 명세 및 입력 서명 |
| `manifest_1050.csv` | 다섯 마모 패턴을 포함한 전체 1,050개 조건 목록 |
| `run_status.csv`, `run_status.mat` | 조건별 상태, 소요 시간 및 오류 |
| `README.md`, `progress.json` | 전체 `완료/1050`, 각 마모 패턴 `완료/210` 진행 상태 및 결과 목록 |
| `run_configuration.mat` | 실행 설정 및 재개 확인용 입력 서명 |
| `runtime_verification.mat` | MEX 사용 전 구현 동등성 검사 통과 기록(검사한 경우) |
| `execution.log` | 셸 실행기로 실행했을 때의 MATLAB 콘솔 로그 |

조건별 MAT에는 시간, 34개 상태, 횡속도·요레이트·롤각·롤/피치 각속도·종속도 등 응답과 바퀴별 힘이 저장됩니다. 기본 10초 결과는 10,001개 시간 샘플을 포함합니다. MAT·CSV·PNG·진행 파일·실행 로그는 Git 추적 대상이며, 컴파일된 가속기와 임시 캐시는 제외합니다.

중단 후 같은 명령을 다시 실행하면 입력 서명이 일치하고 검증을 통과한 결과를 재사용합니다. 모델·데이터·보정·설정을 바꾼 경우 기존 결과의 덮어쓰기를 막습니다. 이때는 다른 `output_dir`를 지정하거나, 덮어쓰기가 필요한 범위를 확인한 후 `overwrite=true`를 명시합니다. 컴파일된 MEX나 실행 폴더 경로가 바뀌어도 서명이 달라질 수 있습니다.

일부 조건이 실패하면 나머지 조건은 계속 처리하고 오류를 상태 파일에 남깁니다. 최종적으로 실패가 남으면 통합 실행기는 오류로 종료합니다. 이 모델의 예측 결과나 MATLAB/C++ 동등성 검사 통과를 새로운 CarSim 조건에 대한 정확도 검증으로 해석하지 않습니다.
