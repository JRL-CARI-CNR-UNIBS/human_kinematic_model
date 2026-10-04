"""Equivalence tests of the JAX model against the C++ model (through the pybind11 bindings)
and against the python translation, which must all return the same results.

Run from the repository root:
    python -m pytest test/python/test_jax_equivalence.py -v

Every JAX test runs on each available device (CPU, and GPU if present), and the FK/IK tests
against each reference implementation (``[cpp]``, ``[python]``). The C++ tests are skipped if
``human_model_binding`` cannot be imported (see conftest.py).

All implementations return NaN for invalid IK solutions (including an out-of-bounds shoulder
rotation) and normalize the chest quaternion in the FK.
"""
import numpy as np
import pytest
import jax
import jax.numpy as jnp

import human_kinematic_model as py_model
import human_kinematic_model_jax as hkm

N_SAMPLES = 300
ATOL_FK = 1e-12
ATOL_IK = 1e-9

PARAM = np.array([0.3, 0.4, 0.25,  # shoulder distance, chest-hip distance, hip distance
                  0.3, 0.3,        # upper and lower arm length
                  0.35, 0.4,       # upper and lower leg length
                  0.4])            # head distance
LIMITS = hkm.default_joint_limits()
LIMB_BLOCKS = {"right_arm": hkm.Q_RIGHT_ARM, "left_arm": hkm.Q_LEFT_ARM,
               "right_leg": hkm.Q_RIGHT_LEG, "left_leg": hkm.Q_LEFT_LEG}

# Reference values, from test_fk_ik.py and test_trunk.py
TEST_Q = np.array([0.680375, -0.211234, 0.566198, 0.485962, 0.670301,
                   -0.492489, -0.268313, 0.536459, -0.444451, 0.10794,
                   -0.0452059, 0.257742, -0.270431, 0.0268018, 0.904459,
                   0.83239, 0.271423, 0.434594, -0.716795, 0.213938,
                   -0.967399, -0.514226, -0.725537, 0.608354, -0.686642,
                   -0.198111, -0.740419, -0.782382])
TEST_KPTS = {
    "head":           [0.687107, -0.544971, 0.345802],
    "left_shoulder":  [0.666024, -0.236367, 0.419017],
    "left_elbow":     [0.862801, -0.298988, 0.636634],
    "left_wrist":     [0.962695, -0.443399, 0.879876],
    "left_hip":       [1.02737, -0.00312208, 0.599881],
    "left_knee":      [1.09435, 0.168946, 0.897213],
    "left_ankle":     [1.09923, 0.340047, 1.25874],
    "right_shoulder": [0.694727, -0.186102, 0.71338],
    "right_elbow":    [0.948553, -0.0811031, 0.592767],
    "right_wrist":    [1.20627, 0.0174684, 0.475016],
    "right_hip":      [1.00407, -0.0997844, 0.829257],
    "right_knee":     [1.11579, 0.225018, 0.896494],
    "right_ankle":    [1.12542, 0.615157, 0.80875],
}
TEST_TRUNK_TFS = {
    "T_ext_rshoulder": [[-0.383699, 0.918489, -0.0956771, 0.694727],
                        [0.915764, 0.365107, -0.167549, -0.186102],
                        [-0.11896, -0.151906, -0.98121, 0.71338],
                        [0, 0, 0, 1]],
    "T_ext_lshoulder": [[-0.383699, 0.918489, -0.0956771, 0.666024],
                        [0.915764, 0.365107, -0.167549, -0.236367],
                        [-0.11896, -0.151906, -0.98121, 0.419017],
                        [0, 0, 0, 1]],
    "T_ext_rhip": [[-0.512902, 0.853371, 0.093214, 1.00407],
                   [0.808482, 0.443688, 0.386649, -0.0997844],
                   [0.288597, 0.273675, -0.917504, 0.829257],
                   [0, 0, 0, 1]],
    "T_ext_lhip": [[-0.512902, 0.853371, 0.093214, 1.02737],
                   [0.808482, 0.443688, 0.386649, -0.00312208],
                   [0.288597, 0.273675, -0.917504, 0.599881],
                   [0, 0, 0, 1]],
    "T_ext_chest": [[-0.383699, 0.387199, -0.838363, 0.680375],
                    [0.915764, 0.0425921, -0.399452, -0.211234],
                    [-0.11896, -0.921012, -0.370925, 0.566198],
                    [0, 0, 0, 1]],
}


def _available_devices():
    devices = jax.devices("cpu")[:1]
    try:
        devices += jax.devices("gpu")[:1]
    except RuntimeError:
        pass
    return devices


DEVICES = _available_devices()


@pytest.fixture(params=DEVICES, ids=lambda d: d.platform)
def device(request):
    return request.param


@pytest.fixture(scope="module")
def cpp():
    return pytest.importorskip("human_model_binding")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def sample_configurations(rng, n, limits=LIMITS):
    """Uniform configurations within the limits, with a normalized chest quaternion (w >= 0),
    as in test_fk_ik.cpp."""
    q = rng.uniform(limits[:, 0], limits[:, 1], size=(n, hkm.N_DOF))
    quat = q[:, 3:7] / np.linalg.norm(q[:, 3:7], axis=1, keepdims=True)
    q[:, 3:7] = np.where(quat[:, 3:4] < 0, -quat, quat)
    return q


def sample_params(rng, n):
    return PARAM * rng.uniform(0.8, 1.2, size=(n, hkm.N_PARAM))


def on(device, *arrays):
    return tuple(jax.device_put(jnp.asarray(a), device) for a in arrays)


def assert_on_device(x, device):
    assert x.devices() == {device}


def assert_allclose_nan(actual, desired, atol):
    """assert_allclose that also requires NaNs in the same positions."""
    actual, desired = np.asarray(actual), np.asarray(desired)
    np.testing.assert_array_equal(np.isnan(actual), np.isnan(desired))
    np.testing.assert_allclose(np.nan_to_num(actual), np.nan_to_num(desired), rtol=0, atol=atol)


class CppReference:
    """C++ model through the pybind11 bindings."""

    def __init__(self, binding):
        self.binding = binding

    def fk(self, q, param):
        return hkm.keypoints_to_array(self.binding.Human28DOF.forward_kinematics(q, param))

    def ik(self, kp, limits, q_previous):
        """IK on a (13, 3) keypoint array and (28, 2) limits."""
        keypoints = self.binding.Keypoints()
        keypoints.set_keypoints(hkm.keypoints_to_dict(np.asarray(kp)))
        joint_limits = [self.binding.JointLimits(lo, hi) for lo, hi in limits]
        return self.binding.Human28DOF.inverse_kinematics(keypoints, joint_limits, q_previous)


class PythonReference:
    """Python translation (scripts/human_kinematic_model.py)."""

    def __init__(self):
        self.model = py_model.HumanProcess()

    def fk(self, q, param):
        return hkm.keypoints_to_array(self.model.forward_kinematics(q, param))

    def ik(self, kp, limits, q_previous):
        """IK on a (13, 3) keypoint array and (28, 2) limits."""
        keypoints = py_model.Keypoints()
        keypoints.set_keypoints(hkm.keypoints_to_dict(np.asarray(kp)))
        joint_limits = [py_model.JointLimits(lo, hi) for lo, hi in limits]
        return self.model.inverse_kinematics(keypoints, joint_limits, q_previous)


@pytest.fixture(params=["cpp", "python"], scope="module")
def reference(request):
    """The implementation the JAX model is compared with."""
    if request.param == "cpp":
        return CppReference(pytest.importorskip("human_model_binding"))
    return PythonReference()


def assert_ik_equal(actual, desired, atol=ATOL_IK):
    """Compare two IK results (q, param, chest_q_rotated), NaNs included."""
    for a, d in zip(actual, desired):
        assert_allclose_nan(a, d, atol)


# ---------------------------------------------------------------------------
# Reference values
# ---------------------------------------------------------------------------

def test_default_joint_limits_match_cpp(cpp):
    np.testing.assert_array_equal(hkm.default_joint_limits(),
                                  hkm.joint_limits_to_array(cpp.Human28DOF.default_joint_limits()))


def test_fk_reference_keypoints(device):
    q, param = on(device, TEST_Q, PARAM)
    kp = jax.jit(hkm.fk)(q, param)
    assert_on_device(kp, device)
    np.testing.assert_allclose(kp, hkm.keypoints_to_array(TEST_KPTS), rtol=0, atol=1e-4)


def test_trunk_fk_reference_transforms(device):
    q, param = on(device, TEST_Q[hkm.Q_TRUNK], PARAM[hkm.P_TRUNK])
    tfs = dict(zip(["T_ext_rshoulder", "T_ext_lshoulder", "T_ext_rhip", "T_ext_lhip", "T_ext_chest"],
                   jax.jit(hkm.trunk_fk)(q, param)))
    for name, expected in TEST_TRUNK_TFS.items():
        np.testing.assert_allclose(tfs[name], expected, rtol=0, atol=1e-4, err_msg=name)


# ---------------------------------------------------------------------------
# JAX vs C++ and python: full FK and IK
# ---------------------------------------------------------------------------

def test_fk_matches_reference(reference, device):
    rng = np.random.default_rng(0)
    q, param = sample_configurations(rng, N_SAMPLES), sample_params(rng, N_SAMPLES)

    kp_jax = hkm.fk_batch(*on(device, q, param))
    assert_on_device(kp_jax, device)

    kp_ref = np.stack([reference.fk(qi, pi) for qi, pi in zip(q, param)])
    np.testing.assert_allclose(kp_jax, kp_ref, rtol=0, atol=ATOL_FK)


def test_ik_roundtrip_matches_reference(reference, device):
    """test_fk_ik.cpp: fk -> ik with the original configuration as previous one -> fk."""
    rng = np.random.default_rng(1)
    q, param = sample_configurations(rng, N_SAMPLES), sample_params(rng, N_SAMPLES)
    kp = np.stack([reference.fk(qi, pi) for qi, pi in zip(q, param)])

    ik_jax = hkm.ik_batch(*on(device, kp, LIMITS, q))
    assert_on_device(ik_jax[0], device)

    for i in range(N_SAMPLES):
        assert_ik_equal([x[i] for x in ik_jax], reference.ik(kp[i], LIMITS, q[i]))

    # both recover the original configuration and parameters
    q_jax, param_jax, _ = ik_jax
    np.testing.assert_allclose(q_jax, q, rtol=0, atol=1e-8)
    np.testing.assert_allclose(param_jax, param, rtol=0, atol=1e-8)
    np.testing.assert_allclose(hkm.fk_batch(q_jax, param_jax), kp, rtol=0, atol=1e-8)


@pytest.mark.parametrize("previous", ["perturbed", "zeros"])
def test_ik_solution_selection_matches_reference(reference, device, previous):
    """With a previous configuration different from the true one, the IK has to pick among
    up to four valid solutions per limb: all implementations must pick the same one."""
    rng = np.random.default_rng(2)
    q, param = sample_configurations(rng, N_SAMPLES), sample_params(rng, N_SAMPLES)
    kp = np.stack([reference.fk(qi, pi) for qi, pi in zip(q, param)])
    q_previous = q + rng.normal(scale=1.0, size=q.shape) if previous == "perturbed" else np.zeros_like(q)

    ik_jax = hkm.ik_batch(*on(device, kp, LIMITS, q_previous))

    n_other_solution = 0
    for i in range(N_SAMPLES):
        ik_ref = reference.ik(kp[i], LIMITS, q_previous[i])
        assert_ik_equal([x[i] for x in ik_jax], ik_ref)
        n_other_solution += not np.allclose(ik_ref[0], q[i], atol=1e-8)

    # the test is only meaningful if the IK actually switched to other solutions
    assert n_other_solution > 0
    np.testing.assert_allclose(hkm.fk_batch(ik_jax[0], ik_jax[1]), kp, rtol=0, atol=1e-8)


def test_ik_invalid_solutions_match_reference(reference, device):
    """Configurations sampled in [-pi, pi] for every joint, often outside the default limits:
    exercises the branches where some IK solutions are rejected, or none is valid (NaN)."""
    rng = np.random.default_rng(3)
    wide_limits = LIMITS.copy()
    wide_limits[7:] = [-np.pi, np.pi]
    q, param = sample_configurations(rng, N_SAMPLES, wide_limits), sample_params(rng, N_SAMPLES)
    kp = np.stack([reference.fk(qi, pi) for qi, pi in zip(q, param)])

    ik_jax = [np.asarray(x) for x in hkm.ik_batch(*on(device, kp, LIMITS, q))]

    n_no_solution = 0
    for i in range(N_SAMPLES):
        ik_ref = reference.ik(kp[i], LIMITS, q[i])
        assert_ik_equal([x[i] for x in ik_jax], ik_ref)
        # when no solution is valid, the whole limb is NaN
        for block in LIMB_BLOCKS.values():
            n_no_solution += np.all(np.isnan(ik_ref[0][block]))
            assert np.all(np.isnan(ik_ref[0][block])) or not np.any(np.isnan(ik_ref[0][block]))

    assert n_no_solution > 0


def test_trunk_ik_out_of_bounds_matches_reference(reference, device):
    """An out-of-bounds shoulder rotation gives NaN for q[7] and both arms, in every implementation."""
    rng = np.random.default_rng(4)
    q, param = sample_configurations(rng, N_SAMPLES), sample_params(rng, N_SAMPLES)
    kp = np.stack([reference.fk(qi, pi) for qi, pi in zip(q, param)])
    tight_limits = LIMITS.copy()
    tight_limits[7] = [-0.5, 0.5]

    ik_jax = [np.asarray(x) for x in hkm.ik_batch(*on(device, kp, tight_limits, q))]

    n_out_of_bounds = 0
    for i in range(N_SAMPLES):
        ik_ref = reference.ik(kp[i], tight_limits, q[i])
        assert_ik_equal([x[i] for x in ik_jax], ik_ref)
        if np.isnan(ik_ref[0][7]):
            n_out_of_bounds += 1
            # the shoulder frames are undefined, so both arms are too; the rest is still computed
            assert np.all(np.isnan(ik_ref[0][hkm.Q_RIGHT_ARM])) and np.all(np.isnan(ik_ref[0][hkm.Q_LEFT_ARM]))
            assert not np.any(np.isnan(ik_ref[0][hkm.Q_RIGHT_LEG]))

    assert 0 < n_out_of_bounds < N_SAMPLES


def test_fk_non_unit_quaternion_matches_reference(reference, device):
    """The chest quaternion is normalized by every FK; a zero quaternion is left unchanged (identity rotation)."""
    rng = np.random.default_rng(15)
    q, param = sample_configurations(rng, 50), sample_params(rng, 50)
    q_scaled = q.copy()
    q_scaled[:, 3:7] *= rng.uniform(0.3, 3.0, size=(50, 1))
    q_zero = q.copy()
    q_zero[:10, 3:7] = 0.0

    kp_unit = hkm.fk_batch(*on(device, q, param))
    kp_scaled = hkm.fk_batch(*on(device, q_scaled, param))
    np.testing.assert_allclose(kp_scaled, kp_unit, rtol=0, atol=ATOL_FK)

    kp_zero = hkm.fk_batch(*on(device, q_zero, param))
    for i in range(50):
        np.testing.assert_allclose(kp_scaled[i], reference.fk(q_scaled[i], param[i]), rtol=0, atol=ATOL_FK)
        np.testing.assert_allclose(kp_zero[i], reference.fk(q_zero[i], param[i]), rtol=0, atol=ATOL_FK)
    q_identity = q_zero[:10].copy()
    q_identity[:, 3:7] = [0.0, 0.0, 0.0, 1.0]
    np.testing.assert_allclose(kp_zero[:10], hkm.fk_batch(q_identity, param[:10]), rtol=0, atol=ATOL_FK)


def test_python_ik_matches_cpp(cpp):
    """Direct python vs C++ check, including other solutions and NaNs."""
    rng = np.random.default_rng(5)
    wide_limits = LIMITS.copy()
    wide_limits[7:] = [-np.pi, np.pi]
    tight_limits = LIMITS.copy()
    tight_limits[7] = [-0.5, 0.5]
    cpp_ref, py_ref = CppReference(cpp), PythonReference()

    for sample_limits, ik_limits in ((LIMITS, LIMITS), (wide_limits, LIMITS), (LIMITS, tight_limits)):
        q, param = sample_configurations(rng, N_SAMPLES, sample_limits), sample_params(rng, N_SAMPLES)
        q_previous = q + rng.normal(scale=1.0, size=q.shape)
        for i in range(N_SAMPLES):
            kp = cpp_ref.fk(q[i], param[i])
            np.testing.assert_allclose(py_ref.fk(q[i], param[i]), kp, rtol=0, atol=ATOL_FK)
            assert_ik_equal(py_ref.ik(kp, ik_limits, q_previous[i]), cpp_ref.ik(kp, ik_limits, q_previous[i]))


# ---------------------------------------------------------------------------
# JAX vs C++ and python: partial functions and transforms
# ---------------------------------------------------------------------------

def test_fk_tfs_matches_cpp(cpp, device):
    rng = np.random.default_rng(6)
    q, param = sample_configurations(rng, 50), sample_params(rng, 50)
    tfs_jax = jax.vmap(hkm.fk_tfs)(*on(device, q, param))

    for i in range(50):
        tfs_cpp = cpp.Human28DOF.forward_kinematics_tfs(q[i], param[i])
        assert set(tfs_cpp) == set(hkm.FkTransforms._fields)
        for name, T_cpp in tfs_cpp.items():
            np.testing.assert_allclose(getattr(tfs_jax, name)[i], T_cpp, rtol=0, atol=ATOL_FK, err_msg=name)


def test_partial_functions_match_cpp(cpp, device):
    """Trunk, head and limb FK/IK functions of the bindings, including full transforms."""
    rng = np.random.default_rng(7)
    H = cpp.Human28DOF
    limits = [cpp.JointLimits(lo, hi) for lo, hi in LIMITS]
    trunk_fk, head_fk = jax.jit(hkm.trunk_fk), jax.jit(hkm.head_fk)
    trunk_ik, head_ik = jax.jit(hkm.trunk_ik), jax.jit(hkm.head_ik)
    limb_fk = {"right": jax.jit(hkm.right_limb_fk), "left": jax.jit(hkm.left_limb_fk)}
    limb_fk_tfs = {"right": jax.jit(hkm.right_limb_fk_tfs), "left": jax.jit(hkm.left_limb_fk_tfs)}
    limb_ik = {"right": jax.jit(hkm.right_limb_ik), "left": jax.jit(hkm.left_limb_ik)}

    for q, param in zip(sample_configurations(rng, 50), sample_params(rng, 50)):
        q_dev, param_dev, limits_dev = on(device, q, param, LIMITS)
        kp = H.forward_kinematics(q, param)

        tfs_cpp = H.trunkFk(q[hkm.Q_TRUNK], param[hkm.P_TRUNK])
        tfs_jax = trunk_fk(q_dev[hkm.Q_TRUNK], param_dev[hkm.P_TRUNK])
        for T_jax, T_cpp in zip(tfs_jax, tfs_cpp):
            np.testing.assert_allclose(T_jax, T_cpp, rtol=0, atol=ATOL_FK)

        np.testing.assert_allclose(head_fk(q_dev[hkm.Q_HEAD], param_dev[hkm.P_HEAD], tfs_jax[4]),
                                   H.headFk(q[hkm.Q_HEAD], param[hkm.P_HEAD], tfs_cpp[4]), rtol=0, atol=ATOL_FK)

        kp_dev = on(device, hkm.keypoints_to_array(kp))[0]
        for x_jax, x_cpp in zip(trunk_ik(kp_dev, limits_dev[7:10]), H.trunkIk(kp, limits[7:10])):
            np.testing.assert_allclose(x_jax, x_cpp, rtol=0, atol=ATOL_IK)
        for x_jax, x_cpp in zip(head_ik(kp_dev, tfs_jax[4], limits_dev[26:28]),
                                H.headIk(kp, tfs_cpp[4], limits[26:28])):
            np.testing.assert_allclose(x_jax, x_cpp, rtol=0, atol=ATOL_IK)

        for side, arm, leg in (("right", hkm.Q_RIGHT_ARM, hkm.Q_RIGHT_LEG), ("left", hkm.Q_LEFT_ARM, hkm.Q_LEFT_LEG)):
            for block, p in ((arm, hkm.P_ARM), (leg, hkm.P_LEG)):
                elbow_cpp, wrist_cpp = getattr(H, f"{side}LimbFk")(q[block], param[p])
                for x_jax, x_cpp in zip(limb_fk[side](q_dev[block], param_dev[p]), (elbow_cpp, wrist_cpp)):
                    np.testing.assert_allclose(x_jax, x_cpp, rtol=0, atol=ATOL_FK)
                for T_jax, T_cpp in zip(limb_fk_tfs[side](q_dev[block], param_dev[p]),
                                        getattr(H, f"{side}LimbFk_tfs")(q[block], param[p])):
                    np.testing.assert_allclose(T_jax, T_cpp, rtol=0, atol=ATOL_FK)
                # IK with a perturbed previous configuration, to exercise the solution selection
                q_previous = q[block] + rng.normal(size=4)
                np.testing.assert_allclose(
                    limb_ik[side](*on(device, elbow_cpp, wrist_cpp), param_dev[p], limits_dev[block],
                                  on(device, q_previous)[0]),
                    getattr(H, f"{side}LimbIk")(elbow_cpp, wrist_cpp, param[p], limits[block], q_previous),
                    rtol=0, atol=ATOL_IK)


def test_partial_fk_matches_python(device):
    """Trunk, head and limb functions, including the full rotation matrices."""
    rng = np.random.default_rng(8)
    model = py_model.HumanProcess()
    trunk_fk = jax.jit(hkm.trunk_fk)
    head_fk = jax.jit(hkm.head_fk)
    right_limb_fk = jax.jit(hkm.right_limb_fk)
    left_limb_fk = jax.jit(hkm.left_limb_fk)

    for q, param in zip(sample_configurations(rng, 50), sample_params(rng, 50)):
        q_dev, param_dev = on(device, q, param)

        tfs_jax = trunk_fk(q_dev[hkm.Q_TRUNK], param_dev[hkm.P_TRUNK])
        tfs_py = model.trunk_fk(q[hkm.Q_TRUNK], param[hkm.P_TRUNK])
        for T_jax, T_py in zip(tfs_jax, tfs_py):
            np.testing.assert_allclose(T_jax, T_py, rtol=0, atol=ATOL_FK)

        np.testing.assert_allclose(head_fk(q_dev[hkm.Q_HEAD], param_dev[hkm.P_HEAD], tfs_jax[4]),
                                   model.head_fk(q[hkm.Q_HEAD], param[7], tfs_py[4]), rtol=0, atol=ATOL_FK)

        for block, p in ((hkm.Q_RIGHT_ARM, hkm.P_ARM), (hkm.Q_RIGHT_LEG, hkm.P_LEG)):
            for jax_fn, py_fn in ((right_limb_fk, model.right_limb_fk), (left_limb_fk, model.left_limb_fk)):
                for p_jax, p_py in zip(jax_fn(q_dev[block], param_dev[p]), py_fn(q[block], param[p])):
                    np.testing.assert_allclose(p_jax, p_py, rtol=0, atol=ATOL_FK)



# ---------------------------------------------------------------------------
# JAX transformations, devices and precision
# ---------------------------------------------------------------------------

def test_batched_matches_unbatched(device):
    rng = np.random.default_rng(9)
    q, param = sample_configurations(rng, 20), sample_params(rng, 20)
    q_dev, param_dev = on(device, q, param)

    kp_batch = hkm.fk_batch(q_dev, param_dev)
    ik_batch = hkm.ik_batch(kp_batch, *on(device, LIMITS, q))
    for i in range(20):
        np.testing.assert_allclose(kp_batch[i], hkm.fk(q[i], param[i]), rtol=0, atol=1e-14)
        for batched, single in zip(ik_batch, hkm.ik(kp_batch[i], LIMITS, q[i])):
            np.testing.assert_allclose(batched[i], single, rtol=0, atol=1e-12)


def test_fk_tfs_consistent_with_fk(device):
    rng = np.random.default_rng(10)
    q, param = sample_configurations(rng, 50), sample_params(rng, 50)
    q_dev, param_dev = on(device, q, param)

    tfs = jax.vmap(hkm.fk_tfs)(q_dev, param_dev)
    kp = hkm.fk_batch(q_dev, param_dev)
    frame_of_keypoint = {"head": "T_ext_head",
                         "left_shoulder": "T_ext_lshoulder", "left_elbow": "T_ext_lelbow",
                         "left_wrist": "T_ext_lwrist", "left_hip": "T_ext_lhip",
                         "left_knee": "T_ext_lknee", "left_ankle": "T_ext_lankle",
                         "right_shoulder": "T_ext_rshoulder", "right_elbow": "T_ext_relbow",
                         "right_wrist": "T_ext_rwrist", "right_hip": "T_ext_rhip",
                         "right_knee": "T_ext_rknee", "right_ankle": "T_ext_rankle"}
    for name, frame in frame_of_keypoint.items():
        np.testing.assert_allclose(getattr(tfs, frame)[:, :3, 3], kp[:, hkm.KP_INDEX[name]],
                                   rtol=0, atol=ATOL_FK, err_msg=name)
    for frame in tfs._fields:
        R = getattr(tfs, frame)[:, :3, :3]
        np.testing.assert_allclose(R @ jnp.swapaxes(R, 1, 2), np.broadcast_to(np.eye(3), R.shape),
                                   rtol=0, atol=1e-12, err_msg=frame)


@pytest.mark.skipif(len(DEVICES) < 2, reason="no GPU available")
def test_cpu_and_gpu_agree():
    rng = np.random.default_rng(11)
    q, param = sample_configurations(rng, 1000), sample_params(rng, 1000)
    cpu, gpu = DEVICES[0], DEVICES[1]

    kp_cpu = hkm.fk_batch(*on(cpu, q, param))
    kp_gpu = hkm.fk_batch(*on(gpu, q, param))
    np.testing.assert_allclose(kp_gpu, kp_cpu, rtol=0, atol=ATOL_FK)

    ik_cpu = hkm.ik_batch(*on(cpu, kp_cpu, LIMITS, q))
    ik_gpu = hkm.ik_batch(*on(gpu, kp_cpu, LIMITS, q))
    for x_gpu, x_cpu in zip(ik_gpu, ik_cpu):
        assert_allclose_nan(x_gpu, x_cpu, ATOL_IK)


def test_float32(device):
    """float32 inputs stay float32 (fast path on consumer GPUs) and match float64 within float32 accuracy.

    A batch of 1000 also catches TF32 matrix products on recent NVIDIA GPUs (~1e-3 m errors).
    """
    rng = np.random.default_rng(12)
    q, param = sample_configurations(rng, 1000), sample_params(rng, 1000)
    q32, param32 = on(device, q.astype(np.float32), param.astype(np.float32))

    kp32 = hkm.fk_batch(q32, param32)
    assert kp32.dtype == jnp.float32
    np.testing.assert_allclose(kp32, hkm.fk_batch(q, param), rtol=0, atol=2e-6)

    q_ik32, param_ik32, chest32 = hkm.ik_batch(kp32, *on(device, LIMITS.astype(np.float32), q32))
    assert q_ik32.dtype == param_ik32.dtype == chest32.dtype == jnp.float32
    kp_roundtrip = hkm.fk_batch(q_ik32, param_ik32)
    assert np.all(np.isfinite(kp_roundtrip))

    # With the shoulder rotation close to +-pi/2 the shoulder line is almost parallel to the chest axis:
    # the chest frame is ill-conditioned and float32 loses ~1e-4, so only check the other samples tightly.
    # The IK joints themselves are ill-conditioned near singular limb poses, hence the keypoint round trip.
    well_conditioned = np.pi / 2 - np.abs(q[:, 7]) > 1e-2
    assert well_conditioned.mean() > 0.9
    np.testing.assert_allclose(np.asarray(param_ik32)[well_conditioned], param[well_conditioned],
                               rtol=0, atol=1e-5)
    np.testing.assert_allclose(np.asarray(kp_roundtrip)[well_conditioned], np.asarray(kp32)[well_conditioned],
                               rtol=0, atol=1e-5)
    np.testing.assert_allclose(kp_roundtrip, kp32, rtol=0, atol=1e-3)


def test_fk_jacobian_matches_finite_differences(device):
    rng = np.random.default_rng(13)
    q, param = sample_configurations(rng, 1)[0], sample_params(rng, 1)[0]
    q_dev, param_dev = on(device, q, param)

    J_q, J_param = jax.jit(jax.jacfwd(hkm.fk, argnums=(0, 1)))(q_dev, param_dev)
    J_q_rev = jax.jit(jax.jacrev(hkm.fk))(q_dev, param_dev)
    np.testing.assert_allclose(J_q, J_q_rev, rtol=0, atol=1e-12)

    eps = 1e-6
    for x, J, fk_of_x in ((q, J_q, lambda x: hkm.fk(x, param)), (param, J_param, lambda x: hkm.fk(q, x))):
        for k in range(x.shape[0]):
            dx = np.zeros_like(x)
            dx[k] = eps
            J_fd = (np.asarray(fk_of_x(x + dx)) - np.asarray(fk_of_x(x - dx))) / (2 * eps)
            np.testing.assert_allclose(J[..., k], J_fd, rtol=0, atol=1e-7)


def test_ik_jacobian_is_finite(device):
    """Gradients through the IK are finite (no NaN leaking from the unselected branches)."""
    rng = np.random.default_rng(14)
    q, param = sample_configurations(rng, 20), sample_params(rng, 20)
    kp = hkm.fk_batch(*on(device, q, param))

    def ik_q_param(kp, q_previous):
        q, param, _ = hkm.ik(kp, LIMITS, q_previous)
        return jnp.concatenate([q, param])

    J = jax.jit(jax.vmap(jax.jacrev(ik_q_param)))(kp, *on(device, q))
    assert J.shape == (20, hkm.N_DOF + hkm.N_PARAM, hkm.N_KEYPOINTS, 3)
    assert np.all(np.isfinite(J))
