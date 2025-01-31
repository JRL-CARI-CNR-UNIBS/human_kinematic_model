import sympy as sym


def right_arm_sym_fk():
    # q1 shoulder rot z
    # q2 shoulder rot x
    # q3 shoulder rot y
    # q4 upper arm length  translation y  (param)
    # q5 elbow rot z
    # q6 lower arm length  translation y (param)
    q1=sym.symbols('q1')    
    q2=sym.symbols('q2')   
    q3=sym.symbols('q3')   
    q4=sym.symbols('q4') #    upper arm length
    q5=sym.symbols('q5')   
    q6=sym.symbols('q6') #    lower arm length


    R01=sym.rot_axis3(-q1) # rot_axis3 is counterclockwise!
    T01=sym.Matrix.vstack(
        sym.Matrix.hstack(R01,sym.zeros(3,1)),
        sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
        )

    R12=sym.rot_axis1(-q2) # rot_axis1 is counterclockwise!
    T12=sym.Matrix.vstack(
        sym.Matrix.hstack(R12,sym.zeros(3,1)),
        sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
        )

    R23=sym.rot_axis2(-q3) # rot_axis2 is counterclockwise!
    T23=sym.Matrix.vstack(
        sym.Matrix.hstack(R23,sym.zeros(3,1)),
        sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
        )

    T34=sym.eye(4)
    T34[1,3]=q4

    T04=T01*T12*T23*T34 # elbow

    print(f"\nELBOW in limb x = {T04[0,3]}")
    print(f"ELBOW in limb y = {T04[1,3]}")
    print(f"ELBOW in limb z = {T04[2,3]}")


    R45=sym.rot_axis3(-q5) # rot_axis3 is counterclockwise!
    T45=sym.Matrix.vstack(
        sym.Matrix.hstack(R45,sym.zeros(3,1)),
        sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
        )

    T56=sym.eye(4)
    T56[1,3]=q6

    T26=T23*T34*T45*T56

    print(f"\nWRIST in frame 2 x = {T26[0,3]}")
    print(f"WRIST in frame 2 y = {T26[1,3]}")
    print(f"WRIST in frame 2 z = {T26[2,3]}")


    T06=T04*T45*T56

    print(f"\nWRIST in limb x = {T06[0,3]}")
    print(f"WRIST in limb y = {T06[1,3]}")
    print(f"WRIST in limb z = {T06[2,3]}")


def main():
    right_arm_sym_fk()


if __name__ == '__main__':
    main()