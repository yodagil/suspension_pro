# 정확한 Octave 실행 환경

이 폴더는 원래 Linux x86-64 MEX 바이트를 그대로 사용하기 위한 설치 도구입니다. 전체 실행 환경(약3.9GB)은 ZIP에 포함하지 않습니다. 고정된 공개 conda-forge 패키지 URL/해시 목록과 micromamba2.10.0 압축 파일을 포함합니다.

프로젝트 루트에서 다음을 실행합니다. `handoff/runtime`은 실제 압축파일 안의 이 폴더 경로로 맞춥니다.

```sh
bash handoff/runtime/setup_octave.sh
bash handoff/runtime/run_octave.sh octave-cli --quiet --no-init-file --no-site-file --no-history handoff/runtime/smoke_runtime.m
```

이미 원래 실행 환경이 있다면 재설치 없이 재사용합니다.

```sh
HMC_OCTAVE_PREFIX=/workspace/work/octave-runtime/env bash handoff/runtime/setup_octave.sh
HMC_OCTAVE_PREFIX=/workspace/work/octave-runtime/env bash handoff/runtime/run_octave.sh octave-cli --quiet --no-init-file --no-site-file --no-history handoff/runtime/smoke_runtime.m
```

`HMC_PROJECT_ROOT`로 프로젝트 위치를, `HMC_RUNTIME_ROOT`로 설치·캐시 위치를 지정할 수 있습니다. 기본 위치는 프로젝트의 `.runtime/`입니다. 설치 위치를 `--prefix`로 바꾸었다면 실행할 때도 `HMC_OCTAVE_PREFIX`를 지정합니다. HOME 변수는 바꾸지 않습니다.

새 환경 설치는 공개 conda-standalone26.9.4.post1(약63MB)을 내려받아 SHA-256을 검사하고, `register_envs:false` 설정으로 `create --file environment-explicit.txt`를 실행합니다. 이는 원래 관리형 환경의 읽기 전용 HOME 때문에 micromamba 환경 등록이 실패했던 문제를 피합니다. micromamba는 기존 환경 실행에 사용합니다. 패키지는 고정 URL과 MD5로 확인하며 솔버로 최신 버전을 다시 고르지 않습니다. 설치에 수GB 여유 공간과 네트워크가 필요합니다.

정확한 패키지 목록에는 x86-64-v4/Ice Lake용 패키지가 있어 AVX-512를 지원하는 Linux x86_64 호스트가 필요합니다. Windows/macOS/ARM이나 다른 Octave 버전에 원래 ELF를 그대로 사용할 수 없습니다. 다른 환경에서 소스 재빌드는 가능하지만 바이너리 SHA가 달라지므로 원래 실행과 동등성 검증·별도 출처 관리 없이 전체 LUT에 섞으면 안 됩니다.

이 패키지의 검증은 **기존 정확한 환경 재사용**, 프로젝트 경로 변경, 원래 MEX의 상태미분/관측값 비트 일치, 초기상태 lift, context526의 H 전체7,381점 일치를 확인했습니다. 수백MB 패키지의 신규 다운로드·설치는 여기서 반복하지 않았으며 bootstrap 설치 분기는 구문 검사만 수행했습니다. `smoke_runtime.m`은 전체 안정영역 궤적을 실행하지 않습니다.
