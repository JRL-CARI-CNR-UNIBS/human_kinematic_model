"""JAX implementation of the 28-DOF human kinematic model.

Pure-function port of the C++ ``human_model::Human28DOF`` class
(``src/human_model/human_model.cpp``), which is the reference implementation.
Every function is compatible with ``jax.jit``, ``jax.vmap`` and ``jax.grad`` and
runs unchanged on CPU or GPU: the device is chosen by where the inputs live
(``jax.device_put(x, jax.devices("gpu")[0])``) or by ``jax.default_device``.

Conventions (identical to the C++ class):
  * configuration ``q`` (28,):
      [0:3]   chest position            [3:7]   chest quaternion (x, y, z, w)
      [7]     shoulder rot x            [8:10]  hip rot z, hip rot x
      [10:14] right arm                 [14:18] left arm
      [18:22] right leg                 [22:26] left leg
      [26:28] head rot x, head rot y
    each limb is (shoulder/hip rot z, rot x, rot y, elbow/knee rot z)
  * parameters ``param`` (8,): shoulder distance, chest-hip distance, hip distance,
    upper arm length, lower arm length, upper leg length, lower leg length, head distance
  * keypoints: (13, 3) array ordered as ``KEYPOINT_NAMES`` (the order of
    ``keypoints::get_keypoints()``)
  * joint limits: (28, 2) array of [min, max]; the checks are strict (min < q < max), except the
    shoulder rotation of the trunk IK, where min <= q <= max

Invalid IK solutions are NaN, as in the C++ class and the python translation: this includes
an out-of-bounds shoulder rotation in ``trunk_ik`` (NaN q[7], which propagates to both arms).
The FK normalizes the chest quaternion (a zero quaternion is left unchanged), as the C++ class.

Enable 64-bit floats (``jax.config.update("jax_enable_x64", True)``) to reproduce
the double-precision C++ results. float32 inputs are supported and keep their dtype.
"""
from functools import reduce
from typing import NamedTuple

import jax
import jax.numpy as jnp
import numpy as np

N_DOF = 28
N_PARAM = 8

KEYPOINT_NAMES = ("head",
                  "left_shoulder", "left_elbow", "left_wrist",
                  "left_hip", "left_knee", "left_ankle",
                  "right_shoulder", "right_elbow", "right_wrist",
                  "right_hip", "right_knee", "right_ankle")
KP_INDEX = {name: i for i, name in enumerate(KEYPOINT_NAMES)}
N_KEYPOINTS = len(KEYPOINT_NAMES)

# Configuration and parameter blocks (same layout as Human28DOF::fk)
Q_TRUNK = slice(0, 10)
Q_RIGHT_ARM = slice(10, 14)
Q_LEFT_ARM = slice(14, 18)
Q_RIGHT_LEG = slice(18, 22)
Q_LEFT_LEG = slice(22, 26)
Q_HEAD = slice(26, 28)

P_TRUNK = slice(0, 3)
P_ARM = slice(3, 5)
P_LEG = slice(5, 7)
P_HEAD = slice(7, 8)


class FkTransforms(NamedTuple):
    """Homogeneous (4, 4) transforms returned by ``fk_tfs``, named as in ``Human28DOF::fk_tfs``."""
    T_ext_rshoulder: jnp.ndarray
    T_ext_lshoulder: jnp.ndarray
    T_ext_rhip: jnp.ndarray
    T_ext_lhip: jnp.ndarray
    T_ext_chest: jnp.ndarray
    T_ext_head: jnp.ndarray
    T_ext_rshoulderRotated: jnp.ndarray
    T_ext_relbow: jnp.ndarray
    T_ext_rwrist: jnp.ndarray
    T_ext_lshoulderRotated: jnp.ndarray
    T_ext_lelbow: jnp.ndarray
    T_ext_lwrist: jnp.ndarray
    T_ext_rhipRotated: jnp.ndarray
    T_ext_rknee: jnp.ndarray
    T_ext_rankle: jnp.ndarray
    T_ext_lhipRotated: jnp.ndarray
    T_ext_lknee: jnp.ndarray
    T_ext_lankle: jnp.ndarray


# ---------------------------------------------------------------------------
# Rotation and transform primitives (Eigen conventions)
# ---------------------------------------------------------------------------

def rot_x(angle):
    """Rotation matrix about x, as Eigen::AngleAxisd(angle, UnitX())."""
    c, s = jnp.cos(angle), jnp.sin(angle)
    one, zero = jnp.ones_like(c), jnp.zeros_like(c)
    return jnp.stack([jnp.stack([one, zero, zero]),
                      jnp.stack([zero, c, -s]),
                      jnp.stack([zero, s, c])])


def rot_y(angle):
    """Rotation matrix about y, as Eigen::AngleAxisd(angle, UnitY())."""
    c, s = jnp.cos(angle), jnp.sin(angle)
    one, zero = jnp.ones_like(c), jnp.zeros_like(c)
    return jnp.stack([jnp.stack([c, zero, s]),
                      jnp.stack([zero, one, zero]),
                      jnp.stack([-s, zero, c])])


def rot_z(angle):
    """Rotation matrix about z, as Eigen::AngleAxisd(angle, UnitZ())."""
    c, s = jnp.cos(angle), jnp.sin(angle)
    one, zero = jnp.ones_like(c), jnp.zeros_like(c)
    return jnp.stack([jnp.stack([c, -s, zero]),
                      jnp.stack([s, c, zero]),
                      jnp.stack([zero, zero, one])])


def matmul(*matrices):
    """Product of the matrices/vectors, left to right, in full precision.

    Without precision=HIGHEST, XLA runs float32 products on recent NVIDIA GPUs in TF32
    (10-bit mantissa), which makes batched float32 keypoints ~1e-3 m wrong.
    """
    return reduce(lambda a, b: jnp.matmul(a, b, precision=jax.lax.Precision.HIGHEST), matrices)


def homogeneous(R, t):
    """(4, 4) homogeneous transform from a rotation R (3, 3) and a translation t (3,)."""
    top = jnp.concatenate([R, t[:, None]], axis=1)
    bottom = jnp.array([[0., 0., 0., 1.]], dtype=R.dtype)
    return jnp.concatenate([top, bottom], axis=0)


def _rotation(R):
    return homogeneous(R, jnp.zeros(3, dtype=R.dtype))


def _translation(t):
    return homogeneous(jnp.eye(3, dtype=t.dtype), t)


def transform_point(T, p):
    """T * p for a homogeneous transform T and a 3D point p."""
    return matmul(T[:3, :3], p) + T[:3, 3]


def inverse_transform_point(T, p):
    """T.inverse() * p for a rigid homogeneous transform T and a 3D point p."""
    return matmul(T[:3, :3].T, p - T[:3, 3])


def quat_to_rotmat(quat):
    """Rotation matrix of a quaternion (x, y, z, w), as Eigen::Quaterniond::toRotationMatrix().

    Like Eigen, the quaternion is not normalized here: use ``normalize_quat`` first for non-unit quaternions.
    """
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]
    tx, ty, tz = 2 * x, 2 * y, 2 * z
    twx, twy, twz = tx * w, ty * w, tz * w
    txx, txy, txz = tx * x, ty * x, tz * x
    tyy, tyz, tzz = ty * y, tz * y, tz * z
    return jnp.stack([jnp.stack([1 - (tyy + tzz), txy - twz, txz + twy]),
                      jnp.stack([txy + twz, 1 - (txx + tzz), tyz - twx]),
                      jnp.stack([txz - twy, tyz + twx, 1 - (txx + tyy)])])


def normalize_quat(quat):
    """Normalized quaternion, as Eigen::Quaterniond::normalize(): a zero quaternion is left unchanged."""
    squared_norm = jnp.sum(quat * quat)
    # safe denominator, so that the gradient is not NaN for a zero quaternion
    return jnp.where(squared_norm > 0, quat / jnp.sqrt(jnp.where(squared_norm > 0, squared_norm, 1)), quat)


def rotmat_to_quat(R):
    """Quaternion (x, y, z, w) of a rotation matrix, as Eigen::Quaterniond(Matrix3d).

    Shoemake's algorithm: all four branches are evaluated and the one Eigen would
    take is selected, so the function stays jit/vmap/grad friendly.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    candidates = []

    # trace > 0
    t = jnp.sqrt(jnp.where(trace > 0, trace + 1, 1))
    f = 0.5 / t
    candidates.append(jnp.stack([(R[2, 1] - R[1, 2]) * f,
                                 (R[0, 2] - R[2, 0]) * f,
                                 (R[1, 0] - R[0, 1]) * f,
                                 0.5 * t]))

    # otherwise, pivot on the largest diagonal element i
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        arg = R[i, i] - R[j, j] - R[k, k] + 1
        t = jnp.sqrt(jnp.where(arg > 0, arg, 1))
        f = 0.5 / t
        quat = [None] * 4
        quat[i] = 0.5 * t
        quat[j] = (R[j, i] + R[i, j]) * f
        quat[k] = (R[k, i] + R[i, k]) * f
        quat[3] = (R[k, j] - R[j, k]) * f
        candidates.append(jnp.stack(quat))

    diag = jnp.diagonal(R)
    i = jnp.where(diag[1] > diag[0], 1, 0)
    i = jnp.where(diag[2] > diag[i], 2, i)
    branch = jnp.where(trace > 0, 0, i + 1)
    return jnp.stack(candidates)[branch]


def quat_multiply(a, b):
    """Hamilton product a * b of quaternions (x, y, z, w), as Eigen's quaternion product."""
    ax, ay, az, aw = a[0], a[1], a[2], a[3]
    bx, by, bz, bw = b[0], b[1], b[2], b[3]
    return jnp.stack([aw * bx + ax * bw + ay * bz - az * by,
                      aw * by + ay * bw + az * bx - ax * bz,
                      aw * bz + az * bw + ax * by - ay * bx,
                      aw * bw - ax * bx - ay * by - az * bz])


def _canonical_quat(quat):
    """Flip the sign of the quaternion so that its scalar part is non-negative."""
    return jnp.where(quat[3] < 0, -quat, quat)


def chest_quat_rotated(chest_q):
    """Chest quaternion rotated by 180 deg about its z axis (Human28DOF::chestQuatRotated)."""
    half_angle = jnp.asarray(0.5 * np.pi, dtype=chest_q.dtype)
    zero = jnp.zeros((), dtype=chest_q.dtype)
    z_half_turn = jnp.stack([zero, zero, jnp.sin(half_angle), jnp.cos(half_angle)])
    return _canonical_quat(quat_multiply(chest_q, z_half_turn))


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def trunk_fk(q_trunk, param_trunk):
    """Trunk frames (Human28DOF::trunkFk).

    Args:
        q_trunk: (10,) chest position, chest quaternion (x, y, z, w), shoulder rot x, hip rot z, hip rot x
        param_trunk: (3,) shoulder distance, chest-hip distance, hip distance

    Returns:
        T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest, each (4, 4)
    """
    q_trunk = jnp.asarray(q_trunk)
    param_trunk = jnp.asarray(param_trunk, dtype=q_trunk.dtype)
    shoulder_rotx, hip_rotz, hip_rotx = q_trunk[7], q_trunk[8], q_trunk[9]
    shoulder_distance, chest_hip_distance, hip_distance = param_trunk[0], param_trunk[1], param_trunk[2]

    minus_half_pi = jnp.asarray(-0.5 * np.pi, dtype=q_trunk.dtype)
    ez = jnp.array([0., 0., 1.], dtype=q_trunk.dtype)

    # the quaternion is normalized, as in the C++ code
    T_ext_chest = homogeneous(quat_to_rotmat(normalize_quat(q_trunk[3:7])), q_trunk[0:3])

    # shoulder frames: z along the line connecting the shoulders, y downwards
    T_ext_shoulder0 = matmul(T_ext_chest, _rotation(rot_x(shoulder_rotx)), _rotation(rot_x(minus_half_pi)))
    T_ext_rshoulder = matmul(T_ext_shoulder0, _translation(-0.5 * shoulder_distance * ez))
    T_ext_lshoulder = matmul(T_ext_shoulder0, _translation(0.5 * shoulder_distance * ez))

    # hip frames: translate down to the hip center, rotate about z and x, then as the shoulders
    T_ext_hip3 = matmul(T_ext_chest,
                        _translation(-chest_hip_distance * ez),
                        _rotation(rot_z(hip_rotz)),
                        _rotation(rot_x(hip_rotx)),
                        _rotation(rot_x(minus_half_pi)))
    T_ext_rhip = matmul(T_ext_hip3, _translation(-0.5 * hip_distance * ez))
    T_ext_lhip = matmul(T_ext_hip3, _translation(0.5 * hip_distance * ez))

    return T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest


def head_fk(q_head, param_head, T_ext_chest):
    """Head frame (Human28DOF::headFk). q_head = (rot x, rot y), param_head = (distance,) or a scalar."""
    q_head = jnp.asarray(q_head)
    distance = jnp.reshape(jnp.asarray(param_head, dtype=q_head.dtype), (-1,))[0]
    ez = jnp.array([0., 0., 1.], dtype=q_head.dtype)
    return matmul(T_ext_chest,
                  _rotation(rot_x(q_head[0])),
                  _rotation(rot_y(q_head[1])),
                  _translation(distance * ez))


def right_limb_fk_tfs(qarm, param):
    """Limb frames in the shoulder/hip frame (Human28DOF::rightLimbFk_tfs).

    Args:
        qarm: (4,) rot z, rot x, rot y, elbow rot z
        param: (2,) upper and lower limb lengths

    Returns:
        T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist, each (4, 4)
    """
    qarm = jnp.asarray(qarm)
    param = jnp.asarray(param, dtype=qarm.dtype)
    ey = jnp.array([0., 1., 0.], dtype=qarm.dtype)
    T_limb_shoulderRotated = _rotation(matmul(rot_z(qarm[0]), rot_x(qarm[1]), rot_y(qarm[2])))
    T_limb_elbow = matmul(T_limb_shoulderRotated, _translation(param[0] * ey))
    T_limb_wrist = matmul(T_limb_elbow, _rotation(rot_z(qarm[3])), _translation(param[1] * ey))
    return T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist


def left_limb_fk_tfs(qarm, param):
    """Left limb frames (Human28DOF::leftLimbFk_tfs): the right ones with the z translation flipped."""
    return tuple(T.at[2, 3].multiply(-1) for T in right_limb_fk_tfs(qarm, param))


def right_limb_fk(qarm, param):
    """Elbow and wrist positions in the shoulder/hip frame (Human28DOF::rightLimbFk)."""
    _, T_limb_elbow, T_limb_wrist = right_limb_fk_tfs(qarm, param)
    return T_limb_elbow[:3, 3], T_limb_wrist[:3, 3]


def _mirror_z(p):
    return p * jnp.array([1., 1., -1.], dtype=p.dtype)


def left_limb_fk(qarm, param):
    """Elbow and wrist positions in the left shoulder/hip frame (Human28DOF::leftLimbFk)."""
    elbow_in_limb, wrist_in_limb = right_limb_fk(qarm, param)
    return _mirror_z(elbow_in_limb), _mirror_z(wrist_in_limb)


def fk(q, param):
    """Forward kinematics (Human28DOF::fk).

    Args:
        q: (28,) configuration
        param: (8,) body parameters

    Returns:
        (13, 3) keypoints in the external frame, ordered as ``KEYPOINT_NAMES``
    """
    q = jnp.asarray(q)
    param = jnp.asarray(param, dtype=q.dtype)

    T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest = trunk_fk(q[Q_TRUNK], param[P_TRUNK])
    T_ext_head = head_fk(q[Q_HEAD], param[P_HEAD], T_ext_chest)

    relbow, rwrist = right_limb_fk(q[Q_RIGHT_ARM], param[P_ARM])
    lelbow, lwrist = left_limb_fk(q[Q_LEFT_ARM], param[P_ARM])
    rknee, rankle = right_limb_fk(q[Q_RIGHT_LEG], param[P_LEG])
    lknee, lankle = left_limb_fk(q[Q_LEFT_LEG], param[P_LEG])

    kp = {"head": T_ext_head[:3, 3],
          "right_shoulder": T_ext_rshoulder[:3, 3],
          "left_shoulder": T_ext_lshoulder[:3, 3],
          "right_hip": T_ext_rhip[:3, 3],
          "left_hip": T_ext_lhip[:3, 3],
          "right_elbow": transform_point(T_ext_rshoulder, relbow),
          "right_wrist": transform_point(T_ext_rshoulder, rwrist),
          "left_elbow": transform_point(T_ext_lshoulder, lelbow),
          "left_wrist": transform_point(T_ext_lshoulder, lwrist),
          "right_knee": transform_point(T_ext_rhip, rknee),
          "right_ankle": transform_point(T_ext_rhip, rankle),
          "left_knee": transform_point(T_ext_lhip, lknee),
          "left_ankle": transform_point(T_ext_lhip, lankle)}
    return jnp.stack([kp[name] for name in KEYPOINT_NAMES])


def fk_tfs(q, param):
    """All body frames in the external frame (Human28DOF::fk_tfs), as an ``FkTransforms``."""
    q = jnp.asarray(q)
    param = jnp.asarray(param, dtype=q.dtype)

    T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest = trunk_fk(q[Q_TRUNK], param[P_TRUNK])
    T_ext_head = head_fk(q[Q_HEAD], param[P_HEAD], T_ext_chest)

    right_arm = right_limb_fk_tfs(q[Q_RIGHT_ARM], param[P_ARM])
    left_arm = left_limb_fk_tfs(q[Q_LEFT_ARM], param[P_ARM])
    right_leg = right_limb_fk_tfs(q[Q_RIGHT_LEG], param[P_LEG])
    left_leg = left_limb_fk_tfs(q[Q_LEFT_LEG], param[P_LEG])

    return FkTransforms(T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest, T_ext_head,
                        *(matmul(T_ext_rshoulder, T) for T in right_arm),
                        *(matmul(T_ext_lshoulder, T) for T in left_arm),
                        *(matmul(T_ext_rhip, T) for T in right_leg),
                        *(matmul(T_ext_lhip, T) for T in left_leg))


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------

def _in_bounds(x, bounds):
    return (x > bounds[0]) & (x < bounds[1])


def _nan_unless(valid, x):
    return jnp.where(valid, x, jnp.nan)


def _div_sin_or_cos(num_if_sin, num_if_cos, angle):
    """num_if_sin / sin(angle) if |sin(angle)| > 0.5, else num_if_cos / cos(angle)."""
    s, c = jnp.sin(angle), jnp.cos(angle)
    use_sin = jnp.abs(s) > 0.5
    return jnp.where(use_sin, num_if_sin, num_if_cos) / jnp.where(use_sin, s, c)


def _shoulder_ik(elbow_in_limb, bounds, first_solution):
    """Shoulder rot z and rot x (Human28DOF::shoulderIk). Returns (q (2,), valid).

    Unlike the C++ code, q is not set to NaN when invalid: NaNs fed to the following computations
    would make the gradients NaN even when a valid solution is selected. Validity is tracked instead.
    """
    e = elbow_in_limb
    if first_solution:
        # hypothesis cos(q2) > 0
        q1 = jnp.arctan2(-e[0], e[1])
    else:
        # hypothesis cos(q2) < 0
        q1 = jnp.arctan2(e[0], -e[1])
    q2 = jnp.arctan2(e[2], _div_sin_or_cos(-e[0], e[1], q1))

    hypothesis = (jnp.cos(q2) > 0) if first_solution else (jnp.cos(q2) < 0)
    valid = _in_bounds(q1, bounds[0]) & _in_bounds(q2, bounds[1]) & hypothesis
    return jnp.stack([q1, q2]), valid


def _elbow_ik(wrist_in_2, upper_length, bounds, first_solution):
    """Shoulder rot y and elbow rot z (Human28DOF::elbowIk). Returns (q (2,), valid), q not NaN-masked."""
    w = wrist_in_2
    q6cosq5 = w[1] - upper_length
    if first_solution:
        # hypothesis sin(q5) > 0
        q3 = jnp.arctan2(w[2], -w[0])
    else:
        # hypothesis sin(q5) < 0
        q3 = jnp.arctan2(-w[2], w[0])
    q5 = jnp.arctan2(_div_sin_or_cos(w[2], -w[0], q3), q6cosq5)

    hypothesis = (jnp.sin(q5) > 0) if first_solution else (jnp.sin(q5) < 0)
    valid = _in_bounds(q3, bounds[0]) & _in_bounds(q5, bounds[1]) & hypothesis
    return jnp.stack([q3, q5]), valid


def _wrist_in_2(q_shoulder, wrist_in_limb):
    """Wrist position in the frame after shoulder rot z and rot x (Human28DOF::computeWristIn2)."""
    return matmul(matmul(rot_z(q_shoulder[0]), rot_x(q_shoulder[1])).T, wrist_in_limb)


def right_limb_ik(elbow_in_limb, wrist_in_limb, param, qarm_bounds, qarm_previous):
    """Closed-form limb IK (Human28DOF::rightLimbIk).

    Up to four solutions are computed (2 for the shoulder x 2 for the elbow); among
    the ones within the joint limits, the closest to ``qarm_previous`` is returned.

    Args:
        elbow_in_limb, wrist_in_limb: (3,) positions in the shoulder/hip frame
        param: (2,) upper and lower limb lengths
        qarm_bounds: (4, 2) joint limits of the limb
        qarm_previous: (4,) previous limb configuration

    Returns:
        (4,) limb configuration, NaN if no solution is valid
    """
    elbow_in_limb = jnp.asarray(elbow_in_limb)
    wrist_in_limb = jnp.asarray(wrist_in_limb, dtype=elbow_in_limb.dtype)
    param = jnp.asarray(param, dtype=elbow_in_limb.dtype)
    qarm_bounds = jnp.asarray(qarm_bounds, dtype=elbow_in_limb.dtype)
    qarm_previous = jnp.asarray(qarm_previous, dtype=elbow_in_limb.dtype)

    candidates, valid = [], []
    for first_shoulder in (True, False):
        q_shoulder, valid_shoulder = _shoulder_ik(elbow_in_limb, qarm_bounds[0:2], first_shoulder)
        wrist_in_2 = _wrist_in_2(q_shoulder, wrist_in_limb)
        for first_elbow in (True, False):
            q_elbow, valid_elbow = _elbow_ik(wrist_in_2, param[0], qarm_bounds[2:4], first_elbow)
            candidates.append(jnp.concatenate([q_shoulder, q_elbow]))
            valid.append(valid_shoulder & valid_elbow)
    candidates = jnp.stack(candidates)
    valid = jnp.stack(valid)

    # Human28DOF::updateIfCloser: candidates are visited in order and replace the current one only
    # if strictly closer, so ties keep the first candidate, which is what argmin does
    distance = jnp.linalg.norm(candidates - qarm_previous, axis=1)
    distance = jnp.where(valid & ~jnp.isnan(distance), distance, jnp.inf)
    best = jnp.argmin(distance)
    return _nan_unless(distance[best] < jnp.inf, candidates[best])


def left_limb_ik(elbow_in_limb, wrist_in_limb, param, qarm_bounds, qarm_previous):
    """Left limb IK (Human28DOF::leftLimbIk): the right one on z-mirrored positions."""
    elbow_in_limb = jnp.asarray(elbow_in_limb)
    wrist_in_limb = jnp.asarray(wrist_in_limb, dtype=elbow_in_limb.dtype)
    return right_limb_ik(_mirror_z(elbow_in_limb), _mirror_z(wrist_in_limb), param, qarm_bounds, qarm_previous)


def trunk_ik(keypoints, qtrunk_bounds):
    """Trunk IK (Human28DOF::trunkIk).

    Args:
        keypoints: (13, 3) keypoints in the external frame
        qtrunk_bounds: (3, 2) limits of shoulder rot x, hip rot z, hip rot x

    Returns:
        q_trunk (10,), param_trunk (3,), chest_q_rotated (4,)
    """
    keypoints = jnp.asarray(keypoints)
    qtrunk_bounds = jnp.asarray(qtrunk_bounds, dtype=keypoints.dtype)
    left_shoulder, right_shoulder = keypoints[KP_INDEX["left_shoulder"]], keypoints[KP_INDEX["right_shoulder"]]
    left_hip, right_hip = keypoints[KP_INDEX["left_hip"]], keypoints[KP_INDEX["right_hip"]]

    upper_chest = 0.5 * (left_shoulder + right_shoulder)
    lower_chest = 0.5 * (left_hip + right_hip)

    shoulder_distance = jnp.linalg.norm(left_shoulder - right_shoulder)
    chest_hip_distance = jnp.linalg.norm(upper_chest - lower_chest)
    hip_distance = jnp.linalg.norm(left_hip - right_hip)

    shoulder_versor_in_ext = (left_shoulder - right_shoulder) / shoulder_distance
    hip_versor_in_ext = (left_hip - right_hip) / hip_distance

    # chest frame: x frontal, y from the right to the left shoulder, z from the lower to the upper chest
    chest_z_in_ext = (upper_chest - lower_chest) / chest_hip_distance
    chest_y_in_ext = shoulder_versor_in_ext - jnp.sum(shoulder_versor_in_ext * chest_z_in_ext) * chest_z_in_ext
    chest_y_in_ext = chest_y_in_ext / jnp.linalg.norm(chest_y_in_ext)
    chest_x_in_ext = jnp.cross(chest_y_in_ext, chest_z_in_ext)
    chest_rot = jnp.stack([chest_x_in_ext, chest_y_in_ext, chest_z_in_ext], axis=1)

    chest_q = _canonical_quat(rotmat_to_quat(chest_rot))
    chest_q_rotated = chest_quat_rotated(chest_q)
    R_ext_chest = quat_to_rotmat(chest_q)

    # shoulder rotation around the chest x axis, NaN if out of bounds
    # (unlike the other checks, the bounds themselves are allowed)
    shoulder_versor_in_chest = matmul(R_ext_chest.T, shoulder_versor_in_ext)
    shoulder_rotx = jnp.arctan2(shoulder_versor_in_chest[2], shoulder_versor_in_chest[1])
    out_of_bounds = (shoulder_rotx < qtrunk_bounds[0, 0]) | (shoulder_rotx > qtrunk_bounds[0, 1])
    shoulder_rotx = _nan_unless(~out_of_bounds, shoulder_rotx)

    # hip rot z and hip rot x
    h = matmul(R_ext_chest.T, hip_versor_in_ext)

    # solution 1: hypothesis cos(hip_rotx) > 0
    hip_rotz_a = jnp.arctan2(-h[0], h[1])
    hip_rotx_a = jnp.arctan2(h[2], _div_sin_or_cos(-h[0], h[1], hip_rotz_a))
    valid_a = ((jnp.cos(hip_rotx_a) > 0)
               & _in_bounds(hip_rotz_a, qtrunk_bounds[1]) & _in_bounds(hip_rotx_a, qtrunk_bounds[2]))

    # solution 2, used only if the first one is not valid: hypothesis cos(hip_rotx) < 0
    hip_rotz_b = jnp.arctan2(h[0], -h[1])
    hip_rotx_b = jnp.arctan2(h[2], _div_sin_or_cos(-h[0], h[1], hip_rotz_b))
    valid_b = ((jnp.cos(hip_rotx_b) < 0)
               & _in_bounds(hip_rotz_b, qtrunk_bounds[1]) & _in_bounds(hip_rotx_b, qtrunk_bounds[2]))

    q_hip = jnp.where(valid_a,
                      jnp.stack([hip_rotz_a, hip_rotx_a]),
                      _nan_unless(valid_b, jnp.stack([hip_rotz_b, hip_rotx_b])))

    q_trunk = jnp.concatenate([upper_chest, chest_q, shoulder_rotx[None], q_hip])
    param_trunk = jnp.stack([shoulder_distance, chest_hip_distance, hip_distance])
    return q_trunk, param_trunk, chest_q_rotated


def head_ik(keypoints, T_ext_chest, qhead_bounds):
    """Head IK (Human28DOF::headIk).

    Args:
        keypoints: (13, 3) keypoints in the external frame
        T_ext_chest: (4, 4) chest frame
        qhead_bounds: (2, 2) limits of head rot x and head rot y

    Returns:
        q_head (2,), param_head (1,)
    """
    keypoints = jnp.asarray(keypoints)
    qhead_bounds = jnp.asarray(qhead_bounds, dtype=keypoints.dtype)
    h = inverse_transform_point(T_ext_chest, keypoints[KP_INDEX["head"]])
    distance = jnp.linalg.norm(h)

    # solution 1: hypothesis cos(q2) > 0
    q1a = jnp.arctan2(-h[1], h[2])
    q2a = jnp.arctan2(h[0], _div_sin_or_cos(-h[1], h[2], q1a))
    valid_a = (jnp.cos(q2a) > 0) & _in_bounds(q1a, qhead_bounds[0]) & _in_bounds(q2a, qhead_bounds[1])

    # solution 2, used only if the first one is not valid: hypothesis cos(q2) < 0
    q1b = jnp.arctan2(h[1], -h[2])
    q2b = jnp.arctan2(h[0], _div_sin_or_cos(-h[1], h[2], q1b))
    valid_b = (jnp.cos(q2b) < 0) & _in_bounds(q1b, qhead_bounds[0]) & _in_bounds(q2b, qhead_bounds[1])

    q_head = jnp.where(valid_a, jnp.stack([q1a, q2a]), _nan_unless(valid_b, jnp.stack([q1b, q2b])))
    return q_head, distance[None]


def ik(keypoints, joint_limits, q_previous):
    """Inverse kinematics (Human28DOF::ik).

    Args:
        keypoints: (13, 3) keypoints in the external frame, ordered as ``KEYPOINT_NAMES``
        joint_limits: (28, 2) joint limits, see ``default_joint_limits``
        q_previous: (28,) previous configuration, used to choose among multiple limb solutions

    Returns:
        q (28,), param (8,), chest_q_rotated (4,)
    """
    keypoints = jnp.asarray(keypoints)
    joint_limits = jnp.asarray(joint_limits, dtype=keypoints.dtype)
    q_previous = jnp.asarray(q_previous, dtype=keypoints.dtype)
    kp = {name: keypoints[i] for i, name in enumerate(KEYPOINT_NAMES)}

    q_trunk, param_trunk, chest_q_rotated = trunk_ik(keypoints, joint_limits[7:10])
    T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest = trunk_fk(q_trunk, param_trunk)

    q_head, param_head = head_ik(keypoints, T_ext_chest, joint_limits[26:28])

    def segment_length(a, b, c, d):
        return 0.5 * (jnp.linalg.norm(kp[a] - kp[b]) + jnp.linalg.norm(kp[c] - kp[d]))

    param_arm = jnp.stack([segment_length("right_elbow", "right_shoulder", "left_elbow", "left_shoulder"),
                           segment_length("right_elbow", "right_wrist", "left_elbow", "left_wrist")])
    param_leg = jnp.stack([segment_length("right_knee", "right_hip", "left_knee", "left_hip"),
                           segment_length("right_knee", "right_ankle", "left_knee", "left_ankle")])

    q_right_arm = right_limb_ik(inverse_transform_point(T_ext_rshoulder, kp["right_elbow"]),
                                inverse_transform_point(T_ext_rshoulder, kp["right_wrist"]),
                                param_arm, joint_limits[Q_RIGHT_ARM], q_previous[Q_RIGHT_ARM])
    q_left_arm = left_limb_ik(inverse_transform_point(T_ext_lshoulder, kp["left_elbow"]),
                              inverse_transform_point(T_ext_lshoulder, kp["left_wrist"]),
                              param_arm, joint_limits[Q_LEFT_ARM], q_previous[Q_LEFT_ARM])
    q_right_leg = right_limb_ik(inverse_transform_point(T_ext_rhip, kp["right_knee"]),
                                inverse_transform_point(T_ext_rhip, kp["right_ankle"]),
                                param_leg, joint_limits[Q_RIGHT_LEG], q_previous[Q_RIGHT_LEG])
    q_left_leg = left_limb_ik(inverse_transform_point(T_ext_lhip, kp["left_knee"]),
                              inverse_transform_point(T_ext_lhip, kp["left_ankle"]),
                              param_leg, joint_limits[Q_LEFT_LEG], q_previous[Q_LEFT_LEG])

    q = jnp.concatenate([q_trunk, q_right_arm, q_left_arm, q_right_leg, q_left_leg, q_head])
    param = jnp.concatenate([param_trunk, param_arm, param_leg, param_head])
    return q, param, chest_q_rotated


# Batched versions: fk_batch(q (B, 28), param (B, 8)), ik_batch(keypoints (B, 13, 3), limits (28, 2), q_previous (B, 28))
fk_batch = jax.jit(jax.vmap(fk))
ik_batch = jax.jit(jax.vmap(ik, in_axes=(0, None, 0)))


# ---------------------------------------------------------------------------
# Host-side helpers
# ---------------------------------------------------------------------------

def default_joint_limits():
    """(28, 2) joint limits of Human28DOF::setDefaultJointLimits."""
    pi = np.pi
    limits = np.tile([-pi, pi], (N_DOF, 1))
    limits[0] = [0.0, 2.0]                 # chest x (not used by the model)
    limits[1] = [-2.5, 2.5]                # chest y (not used by the model)
    limits[2] = [0.8, 1.8]                 # chest z (not used by the model)
    limits[3:7] = [-1.0, 1.0]              # chest quaternion (not used by the model)
    limits[7] = [-pi / 2, pi / 2]          # shoulder rot x
    limits[8] = [-pi / 2, pi / 2]          # hip rot z
    limits[9] = [-pi / 4, pi / 4]          # hip rot x
    limits[10] = [-pi, pi / 2]             # right shoulder rot z
    limits[11] = [-pi, pi / 2]             # right shoulder rot x
    limits[12] = [-0.75 * pi, 0.75 * pi]   # right shoulder rot y
    limits[13] = [-pi, 0.0]                # right elbow rot z
    limits[14] = [-pi, pi / 2]             # left shoulder rot z
    limits[15] = [-pi, pi / 2]             # left shoulder rot x
    limits[16] = [-0.75 * pi, 0.75 * pi]   # left shoulder rot y
    limits[17] = [-pi, 0.0]                # left elbow rot z
    limits[18] = [-pi / 4, 0.75 * pi]      # right hip rot z
    limits[19] = [-pi / 2, pi / 2]         # right hip rot x
    limits[20] = [-0.75 * pi, 0.75 * pi]   # right hip rot y
    limits[21] = [0.0, pi]                 # right knee rot z
    limits[22] = [-pi / 4, 0.75 * pi]      # left hip rot z
    limits[23] = [-pi, pi / 2]             # left hip rot x
    limits[24] = [-0.75 * pi, 0.75 * pi]   # left hip rot y
    limits[25] = [0.0, pi]                 # left knee rot z
    limits[26] = [-pi / 2, pi / 2]         # head rot x
    limits[27] = [-pi / 2, pi / 2]         # head rot y
    return limits


def joint_limits_to_array(joint_limits):
    """(N, 2) array from a list of ``JointLimits`` (binding or python translation)."""
    return np.array([[limit.min, limit.max] for limit in joint_limits], dtype=float)


def keypoints_to_array(keypoints):
    """(13, 3) array from a dict or a ``Keypoints`` object (binding or python translation)."""
    if isinstance(keypoints, dict):
        return np.stack([np.asarray(keypoints[name], dtype=float) for name in KEYPOINT_NAMES])
    return np.stack([np.asarray(getattr(keypoints, name), dtype=float) for name in KEYPOINT_NAMES])


def keypoints_to_dict(keypoints):
    """Dict {name: (3,)} from a (13, 3) keypoint array."""
    return {name: keypoints[i] for i, name in enumerate(KEYPOINT_NAMES)}


def keypoint_distance(kp1, kp2):
    """Sum of the Euclidean distances between corresponding keypoints (keypoints::keypointDistance)."""
    return jnp.sum(jnp.linalg.norm(kp1 - kp2, axis=-1), axis=-1)
