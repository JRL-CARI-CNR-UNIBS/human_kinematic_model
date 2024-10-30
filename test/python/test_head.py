import sympy as sym
import numpy as np
from human_kinematic_model import HumanProcess

Q1 = 0.1433532874090464
Q2 = 0.9446689170495839
D = 0.4
T_EXT_CHEST = np.eye(4)
N_DOF = 7+3+4*4+2
N_PARAM = 3+2+2+1


def test_head_fk():
    # Create human kinematic model
    model = HumanProcess(n_dof=N_DOF, n_params=N_PARAM)

    # Forward kinematics
    T_ext_head = model.head_fk(np.array([Q1, Q2]), D, T_EXT_CHEST)

    # === Symbolic computation ===
    # hip
    q1 = sym.symbols('q1') #head rotx
    q2 = sym.symbols('q2') #head roty
    d  = sym.symbols('d') #head distance

    # Define transformation matrices
    R01 = sym.rot_axis1(-q1)  # rot_axis1 is counterclockwise!
    T01 = sym.Matrix.vstack(
        sym.Matrix.hstack(R01, sym.zeros(3, 1)),
        sym.Matrix([0, 0, 0, 1]).T
    )

    T01_val = T01.subs({q1: Q1})
    print(f"\nT01_val = \n{np.array(T01_val).astype(float)}")

    R12 = sym.rot_axis2(-q2)  # rot_axis2 is counterclockwise!
    T12 = sym.Matrix.vstack(
        sym.Matrix.hstack(R12, sym.zeros(3, 1)),
        sym.Matrix([0, 0, 0, 1]).T
    )

    T12_val = T12.subs({q2: Q2})
    print(f"\nT12_val = \n{np.array(T12_val).astype(float)}")

    T23 = sym.eye(4)
    T23[2, 3] = d

    T23_val = T23.subs({d: D})
    print(f"\nT23_val = \n{np.array(T23_val).astype(float)}")

    T03=T01*T12*T23
    T03_val = np.array(T01_val@T12_val@T23_val).astype(float)

    print(f"\nT03_val = \n{T03_val}")

    print(f"\nHEAD versor y -> x = {T03[0,3]}")
    print(f"HEAD versor y -> y = {T03[1,3]}")
    print(f"HEAD versor y -> z = {T03[2,3]}")

    print(f"\nHEAD versor y -> x = {T03_val[0,3]}")
    print(f"HEAD versor y -> y = {T03_val[1,3]}")
    print(f"HEAD versor y -> z = {T03_val[2,3]}")

    # Assert results
    assert np.allclose(T_ext_head, T03_val, atol=1e-8), \
        "Transformation matrix computed by FK is not equal to symbolic computation"


def main():
    test_head_fk()


if __name__ == '__main__':
    main()