# Human Kinematic Model

A 28-DOF kinematic model of the human body. Forward kinematics (FK) maps a configuration `q` and 8 body
parameters `param` to 13 3D keypoints. Inverse kinematics (IK) maps the keypoints back to `(q, param)` in closed form.

The model comes in three implementations that return the same results (to floating-point rounding, ~1e-13 in double
precision):

| Implementation | Where | Notes |
|---|---|---|
| C++ (reference) | `include/human_model/human_model.hpp`, `src/human_model/human_model.cpp` | `human_model::Human28DOF`, Eigen |
| Python bindings of the C++ class | `src/human_model/bindings.cpp` | module `human_model_binding` (pybind11) |
| Python translation | `scripts/human_kinematic_model.py` | `HumanProcess`, numpy + scipy |
| JAX | `scripts/human_kinematic_model_jax.py` | `jit`/`vmap`/`grad`-compatible, same code on CPU and GPU |

`test/python/test_jax_equivalence.py` checks that they agree.

## Model conventions

- **Configuration `q` (28)**
  - `[0:3]` chest position
  - `[3:7]` chest quaternion `(x, y, z, w)`, scalar last, with `w >= 0`
  - `[7]` shoulder rot x, `[8:10]` hip rot z, hip rot x
  - `[10:14]` right arm, `[14:18]` left arm, `[18:22]` right leg, `[22:26]` left leg; each limb is
    (shoulder/hip rot z, rot x, rot y, elbow/knee rot z)
  - `[26:28]` head rot x, head rot y
- **Parameters `param` (8):** shoulder distance, chest-hip distance, hip distance, upper arm length, lower arm length,
  upper leg length, lower leg length, head distance.
- **Keypoints (13):** head, left shoulder, left elbow, left wrist, left hip, left knee, left ankle, right shoulder,
  right elbow, right wrist, right hip, right knee, right ankle. This is the order of `Keypoints.get_keypoints()` and
  of the `(13, 3)` arrays of the JAX model.
- **Chest frame:** x frontal, y from the right to the left shoulder, z from the lower to the upper chest.
- **Joint limits:** `Human28DOF.default_joint_limits()` (C++/bindings) and `default_joint_limits()` (JAX, as a
  `(28, 2)` array). Entries 0–6 (chest pose) are not used by the model.

### FK and IK behavior

- FK normalizes the chest quaternion, so non-unit quaternions still give a rotation. A zero quaternion is left
  unchanged (identity rotation), as Eigen's `normalize()` does.
- IK takes the previous configuration: `inverse_kinematics(keypoints, joint_limits, q_previous)` returns
  `(q, param, chest_q_rotated)`. `chest_q_rotated` is the chest quaternion rotated by 180° about its z axis.
- Each limb has up to 4 IK solutions (2 for the shoulder × 2 for the elbow). The IK keeps those strictly inside the
  joint limits and returns the one closest to `q_previous`. Hip and head take the first of 2 solutions that is within
  the limits.
- **Invalid solutions are NaN, never exceptions.** A limb with no valid solution has all four joints NaN; an invalid
  hip or head gives NaN angles. A shoulder rotation outside its limits gives NaN `q[7]`, and therefore NaN for both
  arms, while the rest of the IK is still computed.

## Installation

Choose the part you need:

| Use | What to install | Section |
|---|---|---|
| Python package: JAX model, python translation and bindings of the C++ class | `python -m pip install .` | [A](#a-python-package) |
| Bindings only, compiled for one Python environment without installing anything | Eigen, pybind11, one compiler command | [B](#b-python-bindings-for-a-given-environment-without-cmake) |
| C++ library, bindings and tests in a ROS 2 / colcon workspace | apt packages, colcon | [C](#c-c-library-with-colcon) |

> **Always install with `python -m pip`, not `pip`.** `python -m pip` installs into the environment of the `python`
> you run; a bare `pip` can belong to another environment (an alias, `~/.local/bin/pip`, conda) even when the venv is
> active. The packages then land elsewhere and imports fail with `ModuleNotFoundError`. Check with `which python` and
> `python -m pip --version`: both must point into your environment.

### A. Python package

The package `human_model` installs three modules: `human_kinematic_model_jax` (JAX model), `human_kinematic_model`
(python translation) and `human_model_binding` (bindings of the C++ class, compiled during the installation, with
the C++ library linked in). Its dependencies (numpy, scipy, jax) are installed with it.

Requirements: Python >= 3.10 (tested with 3.14) and a C++17 compiler (`sudo apt install g++` on Ubuntu). CMake,
pybind11 and Eigen are fetched by the build when missing: Eigen 3.4.0 is downloaded if the system has none
(`libeigen3-dev`). The binding also needs the Python development headers of the interpreter
(`sudo apt install python3-dev`, or `python3.X-dev` for a Python other than the system default). Without them the
package is installed without `human_model_binding` (a CMake warning, visible with `pip install -v`); the JAX model and
the python translation do not need it. Install the headers and reinstall to add it.

1. **Environment.** Activate the environment that should contain the package (virtualenv, conda, ...), or create a
   virtualenv:
    ```sh
    python3 -m venv --upgrade-deps .venv
    source .venv/bin/activate
    ```
2. **Install**, from this repository:
    ```sh
    python -m pip install .                # CPU build of JAX
    python -m pip install ".[cuda13]"      # NVIDIA GPU, driver >= 580 (nvidia-smi, top right)
    python -m pip install ".[cuda12]"      # NVIDIA GPU, driver >= 525
    python -m pip install ".[test]"        # + pytest and sympy, for test/python
    ```
    Extras can be combined, e.g. `".[cuda13,test]"`. From another folder, give the path instead of `.`; without a
    clone: `python -m pip install "git+https://github.com/JRL-CARI-CNR-UNIBS/human_kinematic_model.git@jax"`.
3. **Check**, from any folder:
    ```sh
    python -c "import jax, human_kinematic_model_jax as hkm, human_model_binding, human_kinematic_model; print(hkm.fk(jax.numpy.zeros(28), jax.numpy.ones(8)).shape, jax.devices())"
    ```
    It should print `(13, 3)` and, with a CUDA extra, a `CudaDevice`.

**Developing the model.** Install it in editable mode, `python -m pip install -e .` (extras as above): the python
modules are then imported from `scripts/` (a `.pth` file), so changes to them are live; the binding is compiled at
installation, so install again after changing the C++ sources. A regular install copies the modules. The tests do not
need the package: `test/python/conftest.py` imports the modules from `scripts/` and the binding from `build/python/`
(section B) or from the installed package.

### B. Python bindings for a given environment (without CMake)

The binding `human_model_binding` (used by the equivalence tests and by the python code that calls the C++ class)
must be compiled for the Python version that imports it. Without CMake and without installing anything, compile it
into the git-ignored `build/` folder; the Python tests look for it in `build/python/`, or in `$HUMAN_MODEL_BINDING_DIR`.

1. System dependencies:
    ```sh
    sudo apt install g++ libeigen3-dev   # libgtest-dev too for the C++ test below
    ```
    pybind11 from pip: `python -m pip install pybind11` (or `-r requirements.txt`).
2. Build, with `PY` the environment's interpreter:
    ```sh
    PY=/path/to/env/bin/python   # e.g. .venv/bin/python
    mkdir -p build/python
    c++ -O3 -DNDEBUG -shared -std=c++17 -fPIC $($PY -m pybind11 --includes) -I include -I /usr/include/eigen3 \
        src/human_model/human_model.cpp src/human_model/bindings.cpp \
        -o build/python/human_model_binding$($PY -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")
    ```
    To import it outside the tests, add `build/python` to `sys.path`; to have it installed, use section A instead.
3. The native benchmark and the gtest can be built in the same way:
    ```sh
    c++ -O3 -DNDEBUG -std=c++17 -I include -I /usr/include/eigen3 \
        src/human_model/human_model.cpp test/benchmark_fk_ik.cpp -o build/benchmark_fk_ik
    c++ -O3 -DNDEBUG -std=c++17 -DPROJECT_SRC_DIRECTORY=\"$PWD\" -I include -I /usr/include/eigen3 \
        src/human_model/human_model.cpp test/test_fk_ik.cpp -o build/human_model_test -lgtest -lpthread
    ```
Rebuild after any change to the C++ sources.

### C. C++ library with colcon

1. **System dependencies** (the rosdep keys are in `package.xml`):
    ```sh
    sudo apt install libeigen3-dev libgtest-dev pybind11-dev
    ```
2. **Create a workspace and clone the repository:**
    ```sh
    mkdir -p ~/projects/ws/src
    cd ~/projects/ws/src
    git clone https://github.com/JRL-CARI-CNR-UNIBS/human_kinematic_model.git
    cd ../..
    ```
3. **Build the package:**
    ```sh
    colcon build --symlink-install --continue-on-error --packages-select human_model
    ```
    This builds the `libhuman_model.so` library, the `human_model_binding` Python module and, with `ENABLE_TESTING`
    (on by default), the `human_model_test` gtest and the `human_model_benchmark` speed test.

    > **Note:** CMake runs `pip install -e` on the repository while *configuring*, with the Python found by CMake
    > (the active virtualenv or conda environment if there is one, otherwise `--user`). Activate the environment you
    > want the package installed in before building (that step is section A).
4. **Global installation only: update `.bashrc`.** Add the install folder to `PYTHONPATH` and `LD_LIBRARY_PATH`, so
   that `human_model_binding` can be imported and finds `libhuman_model.so`:
    ```sh
    export PYTHONPATH="${PYTHONPATH}:${HOME}/projects/ws/install/human_model/lib/human_model/"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HOME}/projects/ws/install/human_model/lib/human_model/"
    ```

### Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `ModuleNotFoundError` (jax, numpy, ...) right after installing | the packages went into another environment: reinstall with `python -m pip` (see the note above) |
| `python -m pip show human_model`: *Package(s) not found* | the installation failed: rerun it and read the end of its output, e.g. `python -m pip install . 2>&1 \| tail -30` |
| installation fails with `CMAKE_CXX_COMPILER not set` / `No CMAKE_CXX_COMPILER could be found` | no C++ compiler: `sudo apt install g++` |
| installation fails while downloading Eigen | no network access to gitlab.com: install Eigen from the system (`sudo apt install libeigen3-dev`) |
| `No module named 'human_model_binding'` after section A | the Python development headers were missing at installation: `sudo apt install python3-dev` (or `python3.X-dev`), then reinstall |
| `No module named 'human_model_binding'` (tests, section B) | the binding is not built for this Python version, or not on `sys.path` |
| `jax.devices()` shows only `CpuDevice` | CPU build of JAX installed, or CUDA build not matching the driver (A.2) |

## Usage

### C++
```cpp
#include <human_model/human_model.hpp>

std::vector<human_model::JointLimits> qbounds;
human_model::Human28DOF::setDefaultJointLimits(qbounds);

human_model::keypoints kp;
human_model::Human28DOF::fk(q, param, kp);

Eigen::VectorXd q2, param2;
Eigen::Vector4d chest_q_rotated;
human_model::Human28DOF::ik(kp, qbounds, q_previous, q2, param2, chest_q_rotated);
```

### Python bindings
Every function returns its outputs. Transforms are 4×4 numpy arrays.
```python
from human_model_binding import Human28DOF, Keypoints

limits = Human28DOF.default_joint_limits()
kp = Human28DOF.forward_kinematics(q, param)                            # -> Keypoints
q2, param2, chest_q_rotated = Human28DOF.inverse_kinematics(kp, limits, q_previous)
tfs = Human28DOF.forward_kinematics_tfs(q, param)                       # {"T_ext_rshoulder": 4x4, ...}, 18 frames
distance, diff = Keypoints.keypoint_distance(kp, kp2)
```
The partial functions are bound too:
- `trunkFk`, `trunkIk`, `headFk`, `headIk`;
- `rightLimbFk`, `leftLimbFk`, `rightLimbFk_tfs`, `leftLimbFk_tfs`;
- `rightLimbIk`, `leftLimbIk`.

The older in-place signatures `forward_kinematics(q, param, kp)` and
`inverse_kinematics(kp, limits, q_previous, q, param, chest_q_rotated)` still work.

### Python translation
```python
from human_kinematic_model import HumanProcess, Keypoints, JointLimits   # with scripts/ on sys.path

model = HumanProcess()
kp = Keypoints()
kp.set_keypoints(model.forward_kinematics(q, param))                   # dict {name: (3,)}
limits = [JointLimits(lo, hi) for lo, hi in default_limits]            # 28 JointLimits
q2, param2, chest_q_rotated = model.inverse_kinematics(kp, limits, q_previous)
```

### JAX
```python
import jax
jax.config.update("jax_enable_x64", True)   # to reproduce the C++ double-precision results
import human_kinematic_model_jax as hkm       # with scripts/ on sys.path

limits = hkm.default_joint_limits()                          # (28, 2)
kp = hkm.fk(q, param)                                        # (13, 3)
q2, param2, chest_q_rotated = hkm.ik(kp, limits, q_previous)

kp_batch = hkm.fk_batch(q_batch, param_batch)                # jitted and vectorized over the first axis
ik_batch = hkm.ik_batch(kp_batch, limits, q_previous_batch)
J = jax.jacfwd(hkm.fk)(q, param)                             # (13, 3, 28)
```
- **Devices.** The same functions run on CPU or GPU, depending on where the inputs are, e.g.
  `jax.device_put(q_batch, jax.devices("gpu")[0])`.
- **Precision.** float32 inputs stay float32 (≈4e-7 m keypoint error); on GPUs this is much faster than float64.
- **Other functions.** `fk_tfs` (18 frames), the partial functions (`trunk_fk`, `trunk_ik`, `head_fk`, `head_ik`,
  `right_limb_fk`, `left_limb_fk`, `right_limb_ik`, `left_limb_ik`, ...), and helpers to convert binding or python
  objects (`keypoints_to_array`, `keypoints_to_dict`, `joint_limits_to_array`).

## Tests

- C++: `human_model_test` (gtest, 10k random FK → IK → FK round trips; it prints every iteration).
- Python, from the repository root:
    ```sh
    python -m pytest test/python/test_jax_equivalence.py -v                            # equivalence of all implementations
    python -m pytest test/python -q --ignore=test/python/test_jax_equivalence.py      # older tests
    ```
  The equivalence tests compare the JAX model with the C++ bindings and with the python translation. They run on
  every available device (CPU, GPU) and cover:
  - FK, and IK round trips;
  - solution selection with a different `q_previous`;
  - NaN cases, and non-unit and zero quaternions;
  - all the partial functions and transforms;
  - float32 inputs, and the FK/IK Jacobians.

  The C++ comparisons are skipped if `human_model_binding` cannot be imported. `test/python/conftest.py` enables JAX
  64-bit floats and puts `scripts/` and `build/python/` on `sys.path`.

## Speedtest

Time per **iteration**, in milliseconds, measured by `python test/python/benchmark_jax.py`. One iteration processes one
configuration `q`:

1. forward kinematics: `(q, param)` → 13 keypoints;
2. inverse kinematics: keypoints → `(q, param)`, with the original `q` as the previous configuration;
3. forward kinematics again, on the IK result.

The configurations are drawn at random within the default joint limits. Each time is an average: over 10k configurations
for native C++ and batched JAX, and over the first 1000 for the loops driven from Python (bindings, pure Python, single JAX
calls).

- **C++ and Python** run one iteration at a time.
- **JAX** is timed in two ways:
  - *single*: one jitted call per configuration, waiting for each result. This is the latency of one iteration, e.g. one
    frame of a filter.
  - *batched*: one jitted call on all 10k configurations at once (`vmap`), divided by 10k. This is the cost per
    configuration when many are processed together.
- The first call of each JAX variant also compiles it (~1–2 s); that call is excluded.

The native row runs `human_model_benchmark` (`build/benchmark_fk_ik` in the direct build; set `$HUMAN_MODEL_BENCHMARK`
to use another executable).

Intel Core Ultra 7 255HX, NVIDIA GeForce RTX 5060 Laptop GPU; C++ built with `-O3`:

| Implementation | Time per iteration |
|---|---|
| C++ (native, Release) | **0.0042 ms** |
| C++ through the Python bindings | **0.0064 ms** |
| Python (pure) | **1.1 ms** |
| JAX, CPU, float64, single | **0.053 ms** |
| JAX, CPU, float32, single | **0.052 ms** |
| JAX, GPU, float64, single | **0.75 ms** |
| JAX, GPU, float32, single | **0.43 ms** |
| JAX, CPU, float64, batched | **0.0081 ms** |
| JAX, CPU, float32, batched | **0.015 ms** |
| JAX, GPU, float64, batched | **0.0014 ms** |
| JAX, GPU, float32, batched | **0.00009 ms** |

A single iteration is fastest in C++, even through the bindings. For one iteration at a time the GPU is the slowest
option, since every call pays the kernel-launch overhead; it only pays off when many configurations are batched together.

The previous figures in this README (10k iterations: C++ Release 4.2 s, Debug 8.3 s, bindings 6.6 s, pure Python 24.0 s)
came from `test_fk_ik.cpp` / `test_fk_ik*.py`. Those print every iteration, so they mostly measured printing.
