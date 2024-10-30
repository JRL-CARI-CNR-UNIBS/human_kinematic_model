import sympy as sym
import numpy as np
from human_kinematic_model import HumanProcess

N_PARAM = 3+2+2+1

shoulder_distance  = 0.3
chest_hip_distance = 0.4
hip_distance       = 0.25
upper_arm_length = 0.3
lower_arm_length = 0.3
upper_leg_length = 0.35
lower_leg_length = 0.4
head_distance    = 0.4

PARAM = np.array([
    shoulder_distance,
    chest_hip_distance,
    hip_distance,
    upper_arm_length,
    lower_arm_length,
    upper_leg_length,
    lower_leg_length,
    head_distance
])

N_DOF = 7+3+4*4+2

# np.random.seed(0)

# Randomly generate configuration vector
Q = np.random.rand(N_DOF)
Q[3:7] /= np.linalg.norm(Q[3:7]) # Normalize chest rotation quaternion


def test_trunk_fk():
    # Create human kinematic model
    model = HumanProcess(n_dof=N_DOF, n_params=N_PARAM)

    # Forward kinematics
    T_ext_rshoulder, T_ext_lshoulder, \
        T_ext_rhip, T_ext_lhip, \
            T_ext_chest = \
                model.trunk_fk(Q[0:10], PARAM[0:3])

    # === Symbolic computation ===
    # Define array composed of N_DOF symbols with given names
    symbol_names = [f'q{i}' for i in range(N_DOF)]
    q = sym.symbols(symbol_names)    

    # Define transformation matrices
    chest_q = sym.Quaternion(a=q[6], b=q[3], c=q[4], d=q[5]) # scalar-first convention
    TExtChest = sym.eye(4)
    TExtChest[:3, :3] = chest_q.to_rotation_matrix()
    TExtChest[:3, 3] = q[0:3]
 
    TExtChest_val = np.array(
        TExtChest.subs({q[i]: Q[i] for i in range(N_DOF)})).astype(float)
    
    print(f"\nTExtChest_val = \n{TExtChest_val}")
    print(f"\nT_ext_chest = \n{T_ext_chest}")

    assert np.allclose(TExtChest_val, T_ext_chest, atol=1e-8), \
        "T_ext_chest matrix computed by FK is not equal to symbolic computation"


    # q0 = sym.symbols('q0') # shoulder_rotx
    # q1 = sym.symbols('q1') # hip_rotz
    # q2 = sym.symbols('q2') # hip_rotx

    # sh_d = sym.symbols('sh_d') # shoulder distance
    # ch_d = sym.symbols('ch_d') # chest-hip distance
    # h_d  = sym.symbols('h_d')  # hip distance

    # R01=sym.rot_axis3(-q1) # rot_axis3 is counterclockwise!
    # T01=sym.Matrix.vstack(
    #     sym.Matrix.hstack(R01,sym.zeros(3,1)),
    #     sym.Matrix([0,0,0,1]).T
    # )

    # R12=sym.rot_axis1(-q2) # rot_axis1 is counterclockwise!
    # T12=sym.Matrix.vstack(
    #     sym.Matrix.hstack(R12,sym.zeros(3,1)),
    #     sym.Matrix([0,0,0,1]).T
    # )

    # T02=T01*T12

    # # print(f"HIP versor x -> x = {T02[0,0]}")
    # # print(f"HIP versor x -> y = {T02[1,0]}")
    # # print(f"HIP versor x -> z = {T02[2,0]}")
    # print(f"HIP versor y -> x = {T02[0,1]}")
    # print(f"HIP versor y -> y = {T02[1,1]}")
    # print(f"HIP versor y -> z = {T02[2,1]}")
    # # print(f"HIP versor z -> x = {T02[0,2]}")
    # # print(f"HIP versor z -> y = {T02[1,2]}")
    # # print(f"HIP versor z -> z = {T02[2,2]}")
    
    
    
    
    
    
    
    
    
    
    
    
    # # hip
    # q1 = sym.symbols('q1') #head rotx
    # q2 = sym.symbols('q2') #head roty
    # d  = sym.symbols('d') #head distance

    # # Define transformation matrices
    # R01 = sym.rot_axis1(-q1)  # rot_axis1 is counterclockwise!
    # T01 = sym.Matrix.vstack(
    #     sym.Matrix.hstack(R01, sym.zeros(3, 1)),
    #     sym.Matrix([0, 0, 0, 1]).T
    # )

    # T01_val = T01.subs({q1: Q1})
    # print(f"\nT01_val = \n{np.array(T01_val).astype(float)}")

    # R12 = sym.rot_axis2(-q2)  # rot_axis2 is counterclockwise!
    # T12 = sym.Matrix.vstack(
    #     sym.Matrix.hstack(R12, sym.zeros(3, 1)),
    #     sym.Matrix([0, 0, 0, 1]).T
    # )

    # T12_val = T12.subs({q2: Q2})
    # print(f"\nT12_val = \n{np.array(T12_val).astype(float)}")

    # T23 = sym.eye(4)
    # T23[2, 3] = d

    # T23_val = T23.subs({d: D})
    # print(f"\nT23_val = \n{np.array(T23_val).astype(float)}")

    # T03=T01*T12*T23
    # T03_val = np.array(T01_val@T12_val@T23_val).astype(float)

    # print(f"\nT03_val = \n{T03_val}")

    # print(f"\nHEAD versor y -> x = {T03[0,3]}")
    # print(f"HEAD versor y -> y = {T03[1,3]}")
    # print(f"HEAD versor y -> z = {T03[2,3]}")

    # print(f"\nHEAD versor y -> x = {T03_val[0,3]}")
    # print(f"HEAD versor y -> y = {T03_val[1,3]}")
    # print(f"HEAD versor y -> z = {T03_val[2,3]}")

    # # Assert results
    # assert np.allclose(T_ext_head, T03_val, atol=1e-8), \
        # "Transformation matrix computed by FK is not equal to symbolic computation"


def main():
    test_trunk_fk()


if __name__ == '__main__':
    main()