# Human Kinematic Model

## Prerequisites
1. If testing is enabled:
    ```sh
    sudo apt-get install libgtest-dev
    ```
2. If bindings are enabled, install pybind11 in the python environment where you want to use this packages's bindings
    ```sh
    conda activate <env_name> # first activate the virtual environment
    conda install -c conda-forge pybind11
    ```
3. Install dependencies
   1. Eigen3
        ```sh
        sudo apt install libeigen3-dev
        ```

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
   
    _The following instructions apply only when performing a global installation of the package!_

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
- Python (bindings): **6.6 s**