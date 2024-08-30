from sympy.matrices import Matrix, eye, zeros, ones, diag, GramSchmidt
import sympy as sym


# hip
q1=sym.var('q1') #spine rotz (T_hip0_hip1)
q2=sym.var('q2') #hips rotx (T_hip1_hip2)


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

T02=T01*T12

# print(f"HIP versor x -> x = {T02[0,0]}")
# print(f"HIP versor x -> y = {T02[1,0]}")
# print(f"HIP versor x -> z = {T02[2,0]}")
print(f"HIP versor y -> x = {T02[0,1]}")
print(f"HIP versor y -> y = {T02[1,1]}")
print(f"HIP versor y -> z = {T02[2,1]}")
# print(f"HIP versor z -> x = {T02[0,2]}")
# print(f"HIP versor z -> y = {T02[1,2]}")
# print(f"HIP versor z -> z = {T02[2,2]}")
