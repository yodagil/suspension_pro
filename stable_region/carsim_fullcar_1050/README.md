# carsim_fullcar 전체1,050조건 안정영역

현재 작업은 원래 **조건 ID1~525**와 **526~1050**으로 분할했습니다. 이 대화는1~525만 계산하고525에서 종료합니다. 후반부는 별도 작업 디렉터리와 `stable-region-526-1050` 브랜치에서 계산합니다. 분담은 [work_split.json](work_split.json), 전반부 실제 진행률은 [shards/first_0001_0525/progress.json](shards/first_0001_0525/progress.json)을 확인하세요. 기존 최상위 실행 상태·진행률은 분할 전 기록입니다.

후반부의 [인계 ZIP과 상세 프롬프트](../handoff/HANDOFF_PROMPT.md)를 제공합니다. 범위별 완료는525/525로 표시하며 전체1,050조건 완료와 구분합니다.

기존 `result/` 1,050회를 생성한 **원래 34상태 full-car 모델**을 사용합니다. 영역 계산은 GNU Octave와 검증된 C++ MEX에서 실행하며, 원래 모델·물리 데이터·보정·의존 소스 8개의 SHA-256을 확인합니다.

| 항목 | 조건 |
| --- | --- |
| 속도 | 50, 60, 70, 80, 90, 100 km/h |
| 마찰계수 | 0.9, 0.8, 0.7, 0.6, 0.5 |
| 조향 명령 | 0, 0.5, 1, 1.5, 2, 2.5, 3° |
| 마모 | [0,0,0,0], [25,25,0,0], [50,50,25,25], [0,0,25,25], [25,25,50,50] |
| 전체 | 5 × 6 × 5 × 7 = **1,050조건**, 마모별 **210조건** |

`case_index.csv`와 `case_index.json`의 상태는 **평형점·입력 준비 상태**입니다. 분할 후 실제 안정영역 완료 수는 각 `shards/<이름>/progress.json`을 확인하고, 실행 상태는 같은 shard의 `runner_state.json`에서 확인합니다. GitHub 파일은 일정 조건 수마다 갱신되는 계산 스냅샷이며, 실시간 프로세스와 시차가 있을 수 있습니다.

## 계산 기준

각 조건에서 자체 수치 평형점을 구하고, 그 평형점의 **실제 평균 전륜 조향각**으로 외부 정적 MF H 기준을 구성합니다(`reference_mode=model_actual_front_trim`). 이는 모든1,050조건에서 같은 원칙입니다. 앞선 세 모델 비교는 CarSim 실제 조향각의 공통 H를 썼으므로, `comparison/`에 저장된 세 비교 지도를 이 배치에 복사하여 재사용하지 않습니다.

초기 횡속도는 -3~3 m/s, 간격0.1 m/s이고, 초기 요레이트는 -1.5~1.5 rad/s, 간격0.025 rad/s입니다. 전체 격자는 **61×121점, 60×120셀**입니다. RK4 시간 간격0.0005초, 관측 간격0.001초로5초·10초 판정을 함께 저장하고, 셀 안정영역은10초 결과를 사용합니다. 통과한 네 꼭짓점과 별도로 적분한 중심점이 모두 통과해야 셀을 승인합니다. 자세한7개 관측값 회복 허용오차와 물리 조건은 상위 [METHODS.md](../METHODS.md)에 있습니다.

자체 평형점이 H 밖이면 주 판정의 필수조건이 실패하므로 주 안정영역은 정의상 빈 집합입니다. 이 경우 결과에 `completion_kind=trim_outside_reference`를 기록하고, 적분하지 않은 보조 동역학 진단을 NaN/unknown으로 남깁니다. 계산 실패를 빈 영역으로 바꾸어 저장하지 않습니다.

고정 격자 경계에 닿은 H와 안정영역에는 각각 잘림 플래그를 저장합니다. 격자 밖이나 H 밖을 시뮬레이션으로 불안정하다고 확인한 것으로 해석하면 안 됩니다. 이는 유한10초·표본 기반 영역이며 연속 불변 안정영역의 증명은 아닙니다.

## 산출물과 진행률

- `regions/case_0001.mat`~`case_1050.mat`: 조건별 마스크, 셀 영역, 평형점, 계산·판정 설정, 원래 모델 서명과 진단값.
- `images/case_0001.png`~`case_1050.png`: 실제 통과 셀 경계와 H, 자체 평형점. 그림의 case번호는 조건 식별자입니다. 최종 사용자 요청에 따라 이미지 생성도 포함합니다.
- `progress.json`: 실제 완료 수`N/1050`, 마모별`N/210`, 실패·남은 조건.
- `case_index.csv`: 원래 `result/`와 일치하는 모든 조건의 순서·마모·속도·마찰·조향·준비 상태.
- `batch_configuration.mat`: 전체 조건, 격자, 판정, 원래 설정 및 배치 입력 서명.
- `run_binding.json`: 실제 실행한 코드·MEX·물리 입력의 고정 해시.
- 모든 조건이 검증된 뒤에만 `carsim_fullcar_stability_LUT_1050.mat`와 `carsim_fullcar_stability_QC_1050.csv`를 생성합니다. 누락된 지도를0으로 채우지 않습니다.

## 실행과 재개

원래 `hmc_mex`와 `stable_region/code/hmc_region_mex`를 현재 플랫폼용으로 준비한 뒤, 저장소 루트에서 실행합니다.

```sh
python stable_region/code/run_region_batch.py --octave octave-cli \
  --start-id 1 --end-id 525 --shard-name first_0001_0525 \
  --publish-branch main --publish-every 25
```

관리형 환경에서 실행 중인 명령은 로컬 Octave 런타임 래퍼를 추가로 사용합니다.

```sh
python stable_region/code/run_region_batch.py \
  --runtime-wrapper /workspace/work/octave-runtime/run-in-env.sh \
  --octave octave-cli --skip-prepare --priority 302,305,307 \
  --start-id 1 --end-id 525 --shard-name first_0001_0525 \
  --publish-branch main --publish-every 25
```

`--skip-prepare`는 전체 입력 준비를 완료했을 때만 사용합니다. 동일 명령을 다시 실행하면 검증된 완료 결과와 부분 체크포인트를 재사용합니다. 코드·바이너리·설정이 고정된 서명과 다르면 이어 계산하지 않습니다. 동시에 두 writer가 실행되지 않도록 잠금을 사용하며, 프로세스 종료 후에는 명령을 다시 실행해 재개할 수 있습니다. 실행 환경이 종료된 동안에는 계산이 진행되지 않습니다.

현4 CPU에서 앞선 세 조건의 실측은 조건당 약4~5분이었으며, 전체 조건에는 더 큰 H가 포함됩니다. 전체 작업은 수일 규모이고 정확한 소요 시간은 각 조건의 실제 궤적 수에 따라 달라집니다. 완료를 표시하려면1,050개 MAT의 조건·마스크·영역·출처를 모두 검증하고 원본 MAT와 연결된 PNG1,050개를 확인해야 합니다.
