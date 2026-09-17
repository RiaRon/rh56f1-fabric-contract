# RH56F1 Fabric 문제 1·2 작업 상태

마지막 갱신: 2026-09-17 (Asia/Seoul)

공개 전달 저장소: https://github.com/RiaRon/rh56f1-fabric-contract

로컬 구현 커밋: sim2real/feature/rh56f1-fabric-contract@8f7888a

## 목적과 범위

RH56F1 Fabric을 실기 배포 경로에서 사용할 수 있도록 다음 두 항목만 구현한다.

1. `sim2real/policy_control`의 RH56F1 `AssetSpec` Fabric 등록 누락 해결
2. 학습/Fabric의 양팔·양손 26 DOF canonical contract와 현재 오른손 미장착 상태의
   active/inactive·parked 매핑 구현

실물 명령, RH56F1 hardware backend, Modbus/register 통신, palm guard/watchdog,
Fabric 생성자 인자 불일치(문제 3), palm frame 전면 정리, gain/dt/decimation 변경은
범위 밖이다.

## 조사한 저장소와 revision

- `sim2real`: `main@9dbc47d8dab6fd66f00971e70319186f27cdf4ac`
- `hdgp`: `main@0fd488b6f7042e13a706e67ac017a768b9645518`
- `urdf`: `main@88f886cc0bf2b092adafa67f03e166fe6df39638`
- `robot_control`: `humble@dcff2a5bd67bdf3cb2416beff45e8843cd8b2b93`

작업 시작 시 위 저장소들은 clean이었고, 변경은 `sim2real`과 이 상태 문서에만
만들었다. 별도 OpenArm outer-command-law 작업공간은 이번 작업에서 제외한다.

## joint-order 근거와 관련 파일

- Fabric 클래스:
  `hdgp/source/FABRICS/src/fabrics_sim/fabrics/openarm_rh56f1_pose_fabric.py`
- Fabric parameter:
  `hdgp/source/FABRICS/src/fabrics_sim/fabric_params/openarm_rh56f1_pose_params.yaml`
- Fabric manifest/URDF:
  `hdgp/source/FABRICS/src/fabrics_sim/models/robots/urdf/openarm_rh56f1/`
- 학습 환경:
  `hdgp/source/openarm/openarm/rh56f1/{right,left}/grasp_v2/`
- Asset manifest/URDF:
  `hdgp/assets/robot/openarm_rh56f1_bi_rl/`
- 배포 registry/contract:
  `sim2real/policy_control/policy_control/contract_assets.py`,
  `sim2real/policy_control/policy_control/contract.py`

Fabric 클래스 주석과 slice 상수, Fabric manifest `cspace_joint_order`, Fabric URDF
revolute document order, HDGP asset manifest `control_joint_order`가 모두 아래 순서로
일치한다.

## canonical 26 DOF joint order

1. `r_aj_1`
2. `r_aj_2`
3. `r_aj_3`
4. `r_aj_4`
5. `r_aj_5`
6. `r_aj_6`
7. `r_aj_7`
8. `r_hj_thumb_1`
9. `r_hj_thumb_2`
10. `r_hj_index_1`
11. `r_hj_middle_1`
12. `r_hj_ring_1`
13. `r_hj_pinky_1`
14. `l_aj_1`
15. `l_aj_2`
16. `l_aj_3`
17. `l_aj_4`
18. `l_aj_5`
19. `l_aj_6`
20. `l_aj_7`
21. `l_hj_thumb_1`
22. `l_hj_thumb_2`
23. `l_hj_index_1`
24. `l_hj_middle_1`
25. `l_hj_ring_1`
26. `l_hj_pinky_1`

그룹/index mapping:

- `right_arm`: `[0, 1, 2, 3, 4, 5, 6]`
- `right_hand`: `[7, 8, 9, 10, 11, 12]`
- `left_arm`: `[13, 14, 15, 16, 17, 18, 19]`
- `left_hand`: `[20, 21, 22, 23, 24, 25]`

## 실제 원인

- `ASSETS["openarm_rh56f1_bi_rl"]` entry는 있었지만 양쪽 `fabric` 값이 모두
  `None`이어서 registry가 실제 Fabric class/URDF/params/world를 제공하지 못했다.
- 기존 계약과 `FabricCore`는 선택한 한쪽 팔이 cspace 첫 7축이고 그 뒤 전체가 그
  손이라고 가정했다. RH56F1 Fabric은 right-first 전역 26축이므로 왼팔 및 양손
  mapping을 안전하게 표현할 수 없었다.
- RH56F1 palm link는 필수 frame인 `{r,l}_hl_palm_sensor`인데 기존 generic 분기가
  `{r,l}_hl_palm`을 선택했다.

## 구현

- RH56F1 registry의 오른쪽/왼쪽 Fabric을 각각
  `OpenArmRh56f1PoseFabric`/`OpenArmRh56f1LeftPoseFabric`으로 등록했다.
- Asset manifest에서 canonical order와 네 그룹을 만들고, index mapping을 joint
  name으로 파생한다. 숫자 index를 별도 소스에 중복 정의하지 않는다.
- 계약에 26개 `home_q`, `parked_q`, position limit, velocity limit, joint group,
  derived index를 기록한다.
- `enabled_groups`는 immutable asset 정보와 분리된 deployment/runtime 설정이다.
  현재 커밋된 계약은 `right_hand: false`, 나머지 세 그룹은 `true`다.
- inactive 그룹은 입력 명령을 무시하고 canonical slot을 `parked_q`로 유지한다.
  `FabricCore`도 각 적분 substep 뒤 inactive slot의 q/qd/qdd를 parked/zero로
  되돌린다.
- 오른손 도착 후 계약 생성 시 `--inactive-groups right_hand`를 제거하거나
  `enabled_groups.right_hand`만 `true`로 바꾼다. 26축 순서와 group index는 변하지
  않는다.
- Fabric `dt=1/60`, decimation `2`, damping/gain/정책 action/checkpoint는 변경하지
  않았다.

## 수정 파일

- `sim2real/policy_control/policy_control/contract.py`
- `sim2real/policy_control/policy_control/contract_assets.py`
- `sim2real/policy_control/policy_control/fabric_core.py`
- `sim2real/policy_control/tools/build_deploy_contract.py`
- `sim2real/logs/policy/asset_openarm_rh56f1_bi_rl/deploy_contract.json`
- `sim2real/tests/policy_control/test_pc_rh56f1_contract.py`
- `sim2real/tests/policy_control/test_pc_fabric_core.py`
- `RH56F1_FABRIC_STATUS.md`

## 테스트

성공:

- `python3 -m pytest -q tests/policy_control/test_pc_rh56f1_contract.py tests/policy_control/test_pc_contract_v2.py`
  - `20 passed, 7 skipped`
- `python3 -m pytest -q tests/policy_control/test_pc_contract.py tests/policy_control/test_pc_contract_v2.py tests/policy_control/test_pc_rh56f1_contract.py`
  - `21 passed, 24 skipped`
- `python3 -m compileall -q policy_control/policy_control policy_control/tools tests/policy_control/test_pc_rh56f1_contract.py tests/policy_control/test_pc_fabric_core.py`
  - 성공
- `git diff --check`
  - 성공

RH56F1 테스트는 registry non-None, 26개/중복 없음, Fabric manifest/URDF/asset manifest
순서 일치, 그룹 크기 7/6/7/6, index 완전 분할, 26개 home/parked/limit, right-hand
inactive parking, 활성 전환 시 order 불변, sentinel split/reassembly, JSON roundtrip을
검사한다.

전체 `tests/policy_control`은 이 호스트에 `torch`, `rclpy`, ROS message/launch,
`h5py`가 없어 전부 실행하지 못했다. 의존 테스트를 제외한 확장 실행은
`315 passed, 119 skipped`였고, 의존성 누락 실패 외에 현재 HDGP DG5F asset과 기존
`palm_alias` 테스트의 선행 불일치 2건이 있었다. 새 의존성은 설치하지 않았다.

## 남은 blocker와 다음 작업

- 문제 3: 현재 공통 `make_fabric`가 RH56F1 생성자에 없는 `robot_dir_name`,
  `robot_name`, `fabric_params_filename`, `default_config_override`,
  `tip_per_finger`, `use_body_repulsion_pairs` 등을 전달한다. 이번 범위에서는
  수정하지 않았으므로 실제 CUDA Fabric 생성은 아직 막혀 있다.
- 이 PC에는 torch/CUDA/Isaac Sim이 없어 실제 Fabric backend와 GPU parity를
  실행하지 못했다.
- 실제 RH56F1 hardware backend, 오른손 캘리브레이션, ROS hardware command는
  구현·실행하지 않았다.
- 다음 작업은 문제 3의 생성자 계약을 별도 수정하고, 같은 26축 계약으로 fake
  backend 및 CUDA 환경에서 parity를 확인하는 것이다.
