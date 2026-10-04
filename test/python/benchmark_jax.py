"""Speed test of the fk -> ik -> fk iteration (as in the README) for every implementation.

One iteration, on one configuration: forward kinematics (q, param -> keypoints), inverse
kinematics (keypoints -> q, param, with the original q as previous configuration), and forward
kinematics again. The time reported is per iteration, in milliseconds.

The native C++ executable (test/benchmark_fk_ik.cpp), the C++ bindings and the python translation
run one iteration at a time. The JAX model is timed in two ways, on each available device, in
float64 and float32:
  * single: one jitted call per configuration (latency, e.g. one frame of a filter);
  * batch: one jitted call on all the configurations (vmap), divided by their number (throughput).

Run from the repository root:
    python test/python/benchmark_jax.py [-n 10000]
The native executable is looked up in $HUMAN_MODEL_BENCHMARK, then build/benchmark_fk_ik
(see CLAUDE.md for the build command); it is skipped if not found.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

# JAX preallocates 75% of the GPU memory by default, which leaves too little to load the kernels
# of all the variants timed here (CUDA_ERROR_OUT_OF_MEMORY)
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import jax

jax.config.update("jax_enable_x64", True)

import numpy as np
import jax.numpy as jnp

sys.path.insert(0, str(Path(__file__).resolve().parent))
import conftest  # noqa: E402,F401  (sets up sys.path)
import human_kinematic_model as py_model  # noqa: E402
import human_kinematic_model_jax as hkm  # noqa: E402
from test_jax_equivalence import PARAM, LIMITS, sample_configurations  # noqa: E402


def time_loop(fn, n_run, n_total):
    """Time fn over n_run configurations, extrapolated to n_total."""
    start = time.perf_counter()
    for i in range(n_run):
        fn(i)
    return (time.perf_counter() - start) * n_total / n_run


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-n", type=int, default=10_000, help="number of configurations (default: 10000)")
    parser.add_argument("--loop-samples", type=int, default=1000,
                        help="configurations actually run by the per-sample loops, extrapolated to -n (default: 1000)")
    args = parser.parse_args()

    q = sample_configurations(np.random.default_rng(0), args.n)
    param = np.tile(PARAM, (args.n, 1))
    n_loop = min(args.loop_samples, args.n)
    results = {}

    native = Path(os.environ.get("HUMAN_MODEL_BENCHMARK", conftest.REPO_ROOT / "build" / "benchmark_fk_ik"))
    if native.is_file():
        output = subprocess.run([str(native), str(args.n)], check=True, capture_output=True, text=True).stdout
        values = dict(line.split() for line in output.splitlines())
        results["C++ (native, loop)"] = float(values["total_seconds"])
    else:
        print(f"{native} not found, skipping the native C++ benchmark")

    try:
        import human_model_binding as cpp
        limits = [cpp.JointLimits(lo, hi) for lo, hi in LIMITS]

        def cpp_step(i):
            kp = cpp.Human28DOF.forward_kinematics(q[i], param[i])
            q2, param2, _ = cpp.Human28DOF.inverse_kinematics(kp, limits, q[i])
            cpp.Human28DOF.forward_kinematics(q2, param2)

        results["C++ (bindings, loop)"] = time_loop(cpp_step, n_loop, args.n)
    except ImportError:
        print("human_model_binding not found, skipping the C++ bindings")

    model = py_model.HumanProcess()
    py_limits = [py_model.JointLimits(lo, hi) for lo, hi in LIMITS]

    def py_step(i):
        kp = py_model.Keypoints()
        kp.set_keypoints(model.forward_kinematics(q[i], param[i]))
        q2, param2, _ = model.inverse_kinematics(kp, py_limits, q[i])
        model.forward_kinematics(q2, param2)

    results["python (loop)"] = time_loop(py_step, n_loop, args.n)

    def iteration(q, param, limits):
        kp = hkm.fk(q, param)
        q2, param2, _ = hkm.ik(kp, limits, q)
        return hkm.fk(q2, param2)

    jax_single = jax.jit(iteration)
    jax_batch = jax.jit(jax.vmap(iteration, in_axes=(0, 0, None)))

    devices = jax.devices("cpu")[:1]
    try:
        devices += jax.devices("gpu")[:1]
    except RuntimeError:
        pass

    for device in devices:
        for dtype in (jnp.float64, jnp.float32):
            q_dev, param_dev, limits_dev = [jax.device_put(jnp.asarray(x, dtype), device) for x in (q, param, LIMITS)]
            label = f"JAX {device.platform} {jnp.dtype(dtype).name}"

            # single: one call per configuration, waiting for each result
            jax_single(q_dev[0], param_dev[0], limits_dev).block_until_ready()  # compilation
            single_inputs = [(q_dev[i], param_dev[i]) for i in range(n_loop)]
            results[f"{label} (single)"] = time_loop(
                lambda i: jax_single(*single_inputs[i], limits_dev).block_until_ready(), n_loop, args.n)

            # batch: one call on all the configurations, best of 5
            start = time.perf_counter()
            jax_batch(q_dev, param_dev, limits_dev).block_until_ready()
            compile_time = time.perf_counter() - start
            runs = []
            for _ in range(5):
                start = time.perf_counter()
                jax_batch(q_dev, param_dev, limits_dev).block_until_ready()
                runs.append(time.perf_counter() - start)
            results[f"{label} (batch)"] = min(runs)
            print(f"{label} (batch): first call (with compilation) {compile_time:.2f} s")

    print(f"\nfk -> ik -> fk, time per iteration ({args.n} configurations; "
          f"one-at-a-time loops measured on {n_loop}):")
    for name, seconds in results.items():
        print(f"  {name:<30} {seconds / args.n * 1e3:10.5f} ms")


if __name__ == "__main__":
    main()
