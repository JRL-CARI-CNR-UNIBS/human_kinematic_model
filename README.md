# Human Kinematic Model

## Installation (using colcon)

1. **Create a workspace**:
    ```sh
    mkdir -p ~/projects/ws/src
    cd ~/projects/ws/src
    ```

2. **Clone the repository**:
    ```sh
    git clone https://github.com/JRL-CARI-CNR-UNIBS/human_kinematic_model.git
    ```

3. **Navigate back to the workspace folder**:
    ```sh
    cd ../..
    ```

4. **Build the package**:
    ```sh
    colcon build --symlink-install --continue-on-error --packages-select human_model
    ```

5. **Update the `.bashrc` file**:

    Add the following lines to update the `PYTHONPATH` and `LD_LIBRARY_PATH`:

    ```sh
    export PYTHONPATH="${PYTHONPATH}:${HOME}/projects/ws/install/human_model/lib/human_model/"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${HOME}/projects/ws/install/human_model/lib/human_model/"
    ```

    This will allow you to import the `human_model_binding` python module and let it find the C++ `libhuman_model.so` shared library.


## Speedtest
Average time taken to run the `test_fk_ik.cpp` or `test_fk_ik.py` or `test_fk_ik_bindings.py` script that executes **10k times** the loop (direct kinematics -> inverse kinematics -> direct kinematics):
- C++ (Release): **4.2 s**
- C++ (Debug): **8.3 s**
- Python (pure): **24.0 s**
- Python (bindings): **6.7 s**