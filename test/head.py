from sympy.matrices import Matrix, eye, zeros, ones, diag, GramSchmidt
import sympy as sym


# hip
q1=sym.var('q1') #head rotx
q2=sym.var('q2') #head roty
d=sym.var('d') #head distance


R01=sym.rot_axis1(-q1) # rot_axis1 is counterclockwise!
T01=sym.Matrix.vstack(
    sym.Matrix.hstack(R01,sym.zeros(3,1)),
    sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
    )

R12=sym.rot_axis2(-q2) # rot_axis2 is counterclockwise!
T12=sym.Matrix.vstack(
    sym.Matrix.hstack(R12,sym.zeros(3,1)),
    sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
    )


T23=sym.eye(4)
T23[2,3]=d

T03=T01*T12*T23

print(f"HEAD versor y -> x = {T03[0,3]}")
print(f"HEAD versor y -> z = {T03[1,3]}")
print(f"HEAD versor y -> z = {T03[2,3]}")
