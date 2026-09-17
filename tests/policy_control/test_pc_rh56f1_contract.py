"""RH56F1 asset registration and its immutable bimanual 26-DOF deployment contract."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from policy_control import contract as C
from policy_control import contract_assets as A


SIM2REAL = Path(__file__).resolve().parents[2]
RL_WS = SIM2REAL.parent
ASSET_NAME = "openarm_rh56f1_bi_rl"
FABRIC_ROOT = RL_WS / "hdgp/source/FABRICS/src/fabrics_sim"
FABRIC_MANIFEST = FABRIC_ROOT / "models/robots/urdf/openarm_rh56f1/openarm_rh56f1_manifest.yaml"
FABRIC_URDF = FABRIC_ROOT / "models/robots/urdf/openarm_rh56f1/openarm_rh56f1.urdf"
COMMITTED_CONTRACT = SIM2REAL / f"logs/policy/asset_{ASSET_NAME}/deploy_contract.json"


@pytest.fixture(scope="module")
def spec():
    return A.asset_spec(ASSET_NAME)


@pytest.fixture(scope="module")
def contract():
    return A.build_asset_contract(ASSET_NAME, enabled_groups={"right_hand": False})


def test_registry_has_both_rh56f1_fabrics(spec):
    assert spec is A.ASSETS[ASSET_NAME]
    assert spec.fabric["right"] is not None and spec.fabric["left"] is not None
    assert spec.fabric["right"].class_name == "OpenArmRh56f1PoseFabric"
    assert spec.fabric["left"].class_name == "OpenArmRh56f1LeftPoseFabric"
    assert all(f.scope == "asset" for f in spec.fabric.values())


def test_canonical_order_matches_asset_fabric_manifest_and_fabric_urdf(spec, contract):
    order = contract.fabric.joint_order
    asset_order = yaml.safe_load(spec.manifest.read_text())["control_joint_order"]
    fabric_order = yaml.safe_load(FABRIC_MANIFEST.read_text())["cspace_joint_order"]
    root = ET.parse(FABRIC_URDF).getroot()
    urdf_order = [joint.get("name") for joint in root.findall("joint")
                  if joint.get("type") in ("revolute", "continuous")]

    assert len(order) == 26
    assert len(set(order)) == 26
    assert order == list(spec.canonical_joint_order) == asset_order == fabric_order == urdf_order
    assert contract.side("right").fabric.joint_order == order
    assert contract.side("left").fabric.joint_order == order


def test_groups_are_complete_disjoint_and_name_derived(contract):
    fabric = contract.fabric
    assert {name: len(joints) for name, joints in fabric.joint_groups.items()} == {
        "right_arm": 7, "right_hand": 6, "left_arm": 7, "left_hand": 6}
    indices = [i for group in fabric.group_indices.values() for i in group]
    assert len(indices) == len(set(indices)) == 26
    assert sorted(indices) == list(range(26))
    assert fabric.group_indices == A.canonical_group_indices(fabric.joint_order, fabric.joint_groups)


def test_static_vectors_and_limits_are_full_length(contract):
    fabric = contract.fabric
    for values in (fabric.home_q, fabric.parked_q, fabric.position_limits, fabric.velocity_limits):
        assert len(values) == 26
    assert all(len(bounds) == 2 and bounds[0] <= bounds[1] for bounds in fabric.position_limits)
    assert all(value > 0 for value in fabric.velocity_limits)
    assert contract.side("right").palm_body == "r_hl_palm_sensor"
    assert contract.side("left").palm_body == "l_hl_palm_sensor"


def test_inactive_right_hand_is_parked_without_changing_canonical_order(contract):
    fabric = contract.fabric
    sentinel = list(range(26))
    groups = A.split_fabric_state(fabric, sentinel)
    parked = A.assemble_fabric_state(fabric, groups, contract.enabled_groups)
    right_hand = fabric.group_indices["right_hand"]
    active = set(range(26)) - set(right_hand)

    assert contract.enabled_groups == {
        "right_arm": True, "right_hand": False, "left_arm": True, "left_hand": True}
    assert [parked[i] for i in right_hand] == [fabric.parked_q[i] for i in right_hand]
    assert all(parked[i] == sentinel[i] for i in active)

    enabled = {name: True for name in fabric.joint_groups}
    activated = A.assemble_fabric_state(fabric, groups, enabled)
    active_contract = A.build_asset_contract(ASSET_NAME, enabled_groups={"right_hand": True})
    assert activated == sentinel
    assert active_contract.enabled_groups["right_hand"] is True
    assert active_contract.fabric.joint_order == fabric.joint_order


def test_sentinel_split_and_reassembly_is_identity(contract):
    sentinel = list(range(26))
    groups = A.split_fabric_state(contract.fabric, sentinel)
    assert A.assemble_fabric_state(contract.fabric, groups) == sentinel


def test_contract_roundtrip_preserves_runtime_group_setting(contract, tmp_path):
    path = tmp_path / "rh56f1.json"
    C.save_contract(contract, path)
    loaded = C.load_contract(path)
    assert loaded == contract
    assert loaded.enabled_groups["right_hand"] is False


def test_committed_deployment_contract_is_the_inactive_right_hand_variant(contract):
    assert C.load_contract(COMMITTED_CONTRACT) == contract


def test_unknown_group_is_rejected():
    with pytest.raises(C.ContractError, match="unknown enabled_groups"):
        A.build_asset_contract(ASSET_NAME, enabled_groups={"right_foot": False})
