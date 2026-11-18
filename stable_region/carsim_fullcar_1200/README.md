# carsim_fullcar 안정영역 1,200조건

<!-- unify-1200-status:start -->
**통합 완료: 1,200/1,200.** MAT·PNG·이미지 바인딩·LUT·QC CSV 검증을 마쳤습니다.
<!-- unify-1200-status:end -->

원래 조향 0~3°의 1,050조건과 조향 3.5°의 추가 150조건을 합칩니다. 속도 50~100 km/h의 6조건, 마찰계수 0.9~0.5의 5조건, 조향각 0~3.5°의 8조건, 마모 5패턴으로 총 1,200조건이며 마모별 240조건입니다.

통합 진행 상태는 마지막 수집 시점의 검증 수입니다. 계산 중인 누락 75조건의 실시간 진행 수는 [담당 shard 진행 상태](../carsim_fullcar_1050/shards/missing_0451_0525/progress.json)에서 확인할 수 있습니다.

- `regions/case_0001.mat` ~ `case_1200.mat`: 실제 계산한 개별 안정영역. 원본 MAT를 바이트 단위로 복사합니다.
- `images/case_0001.png` ~ `case_1200.png` 및 JSON: 원본 이미지와 MAT 연결 해시를 보존합니다.
- `case_index.json`, `input_manifest.json`: 조건, 출처 배치, 입력·결과·이미지 SHA-256, 통합 검증 내역.
- `progress.json`: 실제 검증 완료 수와 마모별 완료 수.
- `carsim_fullcar_stability_LUT_1200.mat`, `carsim_fullcar_stability_QC_1200.csv`, `finalization.json`: 1,200조건 전체 검증이 끝난 뒤 생성합니다.
- `source_batches/`: 서로 다른 두 배치의 입력 인덱스·설정·서명 원본 스냅샷.

원래 JobID 1~1050은 바꾸지 않았습니다. JobID 1051~1200은 조향 3.5° 조건이며, 기존 배치와 추가 배치의 입력 서명을 각각 보존합니다. 원본 ConditionID도 유지하고, 마모별 통합 ConditionID는 기존 1~210 뒤에 추가 211~240을 붙입니다.

기존 `result/`를 계산한 34상태 full-car 모델·보정·MEX를 사용합니다. 영역은 동일한 61×121 정점 격자에서 계산하며 5초·10초 판정과 실제 셀 중심 검증을 보존합니다. MATLAB 원본의 GNU Octave 10.3 호환 실행 결과입니다. 복사·통합 과정에서 영역 보간, convex hull, 이미지 재생성은 하지 않습니다.

저장소 루트에서 다음 명령으로 기존 결과의 수치·출처·이미지 연결을 검사하고 통합할 수 있습니다. 전체 검증은 1,200조건이 모두 있어야 통과합니다.

```bash
python stable_region/code/consolidate_regions_1200.py --root .
```

미완료 상태에서 검증된 파일만 모으려면 `--collect-available`을 사용합니다. 이 모드는 전체 LUT나 완료 선언을 만들지 않습니다. 최종 완료 이후에는 전체 검증 명령을 사용하세요.
