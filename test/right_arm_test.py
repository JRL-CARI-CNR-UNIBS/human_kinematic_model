from sympy.matrices import Matrix, eye, zeros, ones, diag, GramSchmidt
import sympy as sym


q1=sym.var('q1') 
q2=sym.var('q2')
q3=sym.var('q3')
q4=sym.var('q4') # upper arm length
q5=sym.var('q5')
q6=sym.var('q6') # lower arm length



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
print(f"ELBOW x = {T04[0,3]}")
print(f"ELBOW y = {T04[1,3]}")
print(f"ELBOW z = {T04[2,3]}")
R45=sym.rot_axis3(-q5) # rot_axis3 is counterclockwise!
T45=sym.Matrix.vstack(
    sym.Matrix.hstack(R45,sym.zeros(3,1)),
    sym.Matrix.transpose(sym.Matrix([0,0,0,1]))
    )


T56=sym.eye(4)
T56[1,3]=q6
T26=T23*T34*T45*T56
print(f"WRIST x = {T26[0,3]}")
print(f"WRIST y = {T26[1,3]}")
print(f"WRIST z = {T26[2,3]}")
