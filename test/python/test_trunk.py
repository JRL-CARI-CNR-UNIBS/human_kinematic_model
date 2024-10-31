import copy
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

# Randomly generate configuration vector
Q = np.random.rand(N_DOF)
Q[3:7] /= np.linalg.norm(Q[3:7]) # Normalize chest rotation quaternion

Q_TRUNK_TEST = \
    np.array([
         0.680375, -0.211234, 0.566198,  0.485962, 0.670301,
        -0.492489, -0.268313, 0.536459, -0.444451, 0.10794
    ])

PARAM_TRUNK_TEST = PARAM[0:3]

T_EXT_RSHOULDER_TEST = \
    np.array([
        -0.383699,  0.918489, -0.0956771,  0.694727,
         0.915764,  0.365107, -0.167549 , -0.186102,
        -0.11896 , -0.151906, -0.98121  ,  0.71338 ,
         0       ,  0       ,  0        ,  1
    ])

T_EXT_LSHOULDER_TEST = \
    np.array([
        -0.383699,  0.918489, -0.0956771,  0.666024,
         0.915764,  0.365107, -0.167549 , -0.236367,
        -0.11896 , -0.151906, -0.98121  ,  0.419017,
         0       ,  0       ,  0        ,  1
    ])

T_EXT_RHIP_TEST = \
    np.array([
        -0.512902,  0.853371,  0.093214,  1.00407  ,
         0.808482,  0.443688,  0.386649, -0.0997844,
         0.288597,  0.273675, -0.917504,  0.829257 ,
         0       ,  0       ,  0       ,  1
    ])

T_EXT_LHIP_TEST = \
    np.array([
        -0.512902,  0.853371,  0.093214,  1.02737  ,
         0.808482,  0.443688,  0.386649, -0.00312208,
         0.288597,  0.273675, -0.917504,  0.599881 ,
         0       ,  0       ,  0       ,  1
    ])

T_EXT_CHEST_TEST = \
    np.array([
        -0.383699,  0.387199 , -0.838363,  0.680375,
         0.915764,  0.0425921, -0.399452, -0.211234,
        -0.11896 , -0.921012 , -0.370925,  0.566198,
         0       ,  0        ,  0       ,  1
    ])


def test_trunk_fk():
    # Create human kinematic model
    model = HumanProcess(n_dof=N_DOF, n_params=N_PARAM)

    # Forward kinematics
    T_ext_rshoulder, T_ext_lshoulder, \
        T_ext_rhip, T_ext_lhip, \
            T_ext_chest = \
                model.trunk_fk(Q_TRUNK_TEST, PARAM_TRUNK_TEST)
    
    assert np.allclose(T_EXT_RSHOULDER_TEST, T_ext_rshoulder.flatten(), atol=1e-4), \
        "T_ext_rshoulder matrix computed by FK is not equal to the test value"
    
    assert np.allclose(T_EXT_LSHOULDER_TEST, T_ext_lshoulder.flatten(), atol=1e-4), \
        "T_ext_lshoulder matrix computed by FK is not equal to the test value"
    
    assert np.allclose(T_EXT_RHIP_TEST, T_ext_rhip.flatten(), atol=1e-4), \
        "T_ext_rhip matrix computed by FK is not equal to the test value"
    
    assert np.allclose(T_EXT_LHIP_TEST, T_ext_lhip.flatten(), atol=1e-4), \
        "T_ext_lhip matrix computed by FK is not equal to the test value"
    
    assert np.allclose(T_EXT_CHEST_TEST, T_ext_chest.flatten(), atol=1e-4), \
        "T_ext_chest matrix computed by FK is not equal to the test value"


def test_trunk_sym_fk():
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

    # Define array of parameters based on the names in PARAM
    param_names = ['shoulder_distance', 'chest_hip_distance', 'hip_distance',
                   'upper_arm_length', 'lower_arm_length', 'upper_leg_length',
                   'lower_leg_length', 'head_distance']
    param = sym.symbols(param_names)

    # Define a dictionary with the symbols and parameters used for substitution
    q_param_dict = {**{q[i]: Q[i] for i in range(N_DOF)},
                    **{param[i]: PARAM[i] for i in range(N_PARAM)}}


    # CHEST REFERENCE FRAME:
    chest_q = sym.Quaternion(a=q[6], b=q[3], c=q[4], d=q[5]) # scalar-first convention
    TExtChest = sym.eye(4)
    TExtChest[:3, :3] = chest_q.to_rotation_matrix()
    TExtChest[:3, 3] = q[0:3]
 
    TExtChest_val = np.array(
        TExtChest.subs(q_param_dict)).astype(float)
    
    print(f"\nTExtChest_val = \n{TExtChest_val}")
    print(f"\nT_ext_chest = \n{T_ext_chest}")

    assert np.allclose(TExtChest_val, T_ext_chest, atol=1e-8), \
        "T_ext_chest matrix computed by FK is not equal to symbolic computation"


    # ROTATED CHEST REFERENCE FRAME:
    shoulder_rotx = q[7]
    TChestShoulder = sym.eye(4)
    TChestShoulder[:3, :3] = sym.rot_axis1(-shoulder_rotx) # "-" because sympy expresses rotations in opposite direction


    # RIGHT SHOULDER REFERENCE FRAME:
    # Rotation around x-axis
    TShoulderRshoulder0 = sym.eye(4)
    TShoulderRshoulder0[:3, :3] = sym.rot_axis1(sym.pi/2) # "-" because sympy expresses rotations in opposite direction

    # Translation along z-axis
    TRshoulder0Rshoulder = sym.eye(4)
    TRshoulder0Rshoulder[2, 3] = -0.5 * param[0]

    # Combine the transformations
    TExtRshoulder = TExtChest @ TChestShoulder @ TShoulderRshoulder0 @ TRshoulder0Rshoulder
    TExtRshoulder_val = np.array(
        TExtRshoulder.subs(q_param_dict)).astype(float)
    
    print(f"\nTExtRshoulder_val = \n{TExtRshoulder_val}")
    print(f"\nT_ext_rshoulder = \n{T_ext_rshoulder}")

    assert np.allclose(TExtRshoulder_val, T_ext_rshoulder, atol=1e-8), \
        "T_ext_rshoulder matrix computed by FK is not equal to symbolic computation"


    # LEFT SHOULDER REFERENCE FRAME:
    # Rotation around x-axis
    TShoulderLshoulder0 = sym.eye(4)
    TShoulderLshoulder0[:3, :3] = sym.rot_axis1(sym.pi/2) # "-" because sympy expresses rotations in opposite direction

    # Translation along z-axis
    TLshoulder0Lshoulder = sym.eye(4)
    TLshoulder0Lshoulder[2, 3] = 0.5 * param[0]

    # Combine the transformations
    TExtLshoulder = TExtChest @ TChestShoulder @ TShoulderLshoulder0 @ TLshoulder0Lshoulder
    TExtLshoulder_val = np.array(
        TExtLshoulder.subs(q_param_dict)).astype(float)
    
    print(f"\nTExtLshoulder_val = \n{TExtLshoulder_val}")
    print(f"\nT_ext_lshoulder = \n{T_ext_lshoulder}")

    assert np.allclose(TExtLshoulder_val, T_ext_lshoulder, atol=1e-8), \
        "T_ext_lshoulder matrix computed by FK is not equal to symbolic computation"
    

    # HIP REFERENCE FRAME:
    TChestHip0 = sym.eye(4)
    TChestHip0[2, 3] = -param[1]

    # TWICE-RORATED HIP REFERENCE FRAME:
    hip_rotz = q[8]
    THip0Hip1 = sym.eye(4)
    THip0Hip1[:3, :3] = sym.rot_axis3(-hip_rotz) # "-" because sympy expresses rotations in opposite direction

    hip_rotx = q[9]
    THip1Hip2 = sym.eye(4)
    THip1Hip2[:3, :3] = sym.rot_axis1(-hip_rotx) # "-" because sympy expresses rotations in opposite direction

    # RIGHT HIP REFERENCE FRAME:
    THip2Rhip3 = sym.eye(4)
    THip2Rhip3[:3, :3] = sym.rot_axis1(sym.pi/2) # "-" because sympy expresses rotations in opposite direction

    TRhip3Rhip = sym.eye(4)
    TRhip3Rhip[2, 3] = -0.5 * param[2]

    # Combine the transformations
    TExtRhip = TExtChest @ TChestHip0 @ THip0Hip1 @ THip1Hip2 @ THip2Rhip3 @ TRhip3Rhip

    TExtRhip_val = np.array(
        TExtRhip.subs(q_param_dict)).astype(float)
    
    print(f"\nTExtRhip_val = \n{TExtRhip_val}")
    print(f"\nT_ext_rhip = \n{T_ext_rhip}")

    assert np.allclose(TExtRhip_val, T_ext_rhip, atol=1e-8), \
        "T_ext_rhip matrix computed by FK is not equal to symbolic computation"
    

    # LEFT HIP REFERENCE FRAME:
    THip2Lhip3 = copy.copy(THip2Rhip3)

    TLhip3Lhip = sym.eye(4)
    TLhip3Lhip[2, 3] = 0.5 * param[2]

    # Combine the transformations
    TExtLhip = TExtChest @ TChestHip0 @ THip0Hip1 @ THip1Hip2 @ THip2Lhip3 @ TLhip3Lhip

    TExtLhip_val = np.array(
        TExtLhip.subs(q_param_dict)).astype(float)
    
    print(f"\nTExtLhip_val = \n{TExtLhip_val}")
    print(f"\nT_ext_lhip = \n{T_ext_lhip}")

    assert np.allclose(TExtLhip_val, T_ext_lhip, atol=1e-8), \
        "T_ext_lhip matrix computed by FK is not equal to symbolic computation"


def truck_sym_fk():
    q0 = sym.symbols('q0') # shoulder_rotx
    q1 = sym.symbols('q1') # hip_rotz
    q2 = sym.symbols('q2') # hip_rotx

    sh_d = sym.symbols('sh_d') # shoulder distance
    ch_d = sym.symbols('ch_d') # chest-hip distance
    h_d  = sym.symbols('h_d')  # hip distance

    R01=sym.rot_axis3(-q1) # rot_axis3 is counterclockwise!
    T01=sym.Matrix.vstack(
        sym.Matrix.hstack(R01,sym.zeros(3,1)),
        sym.Matrix([0,0,0,1]).T
    )

    R12=sym.rot_axis1(-q2) # rot_axis1 is counterclockwise!
    T12=sym.Matrix.vstack(
        sym.Matrix.hstack(R12,sym.zeros(3,1)),
        sym.Matrix([0,0,0,1]).T
    )

    T02=T01*T12

    print(f"HIP versor x -> x = {T02[0,0]}")
    print(f"HIP versor x -> y = {T02[1,0]}")
    print(f"HIP versor x -> z = {T02[2,0]}")
    print(f"HIP versor y -> x = {T02[0,1]}")
    print(f"HIP versor y -> y = {T02[1,1]}")
    print(f"HIP versor y -> z = {T02[2,1]}")
    print(f"HIP versor z -> x = {T02[0,2]}")
    print(f"HIP versor z -> y = {T02[1,2]}")
    print(f"HIP versor z -> z = {T02[2,2]}")


def main():
    test_trunk_fk()


if __name__ == '__main__':
    main()