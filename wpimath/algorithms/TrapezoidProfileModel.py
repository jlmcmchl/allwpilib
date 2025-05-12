from sympy import *
from sympy.logic.boolalg import *

init_printing()

U, A, B, t, x0, xf, v0, vf, c1, c2, v, V, kV, kA, vmax, a, a1, a2 = symbols(
    "U, A, B t, x0, xf, v0, vf, C1, C2, v, V, kV, kA, vmax, a, a1, a2"
)

x = symbols("x", cls=Function)

# Trapezoid profiles are derived from a differential equation: ẍ = a
diffeq = Eq(x(t).diff(t, t), a)

x = dsolve(diffeq).rhs
dx = x.diff(t)

x = x.subs(
    [
        (c1, solve(Eq(x.subs(t, 0), x0), c1)[0]),
        (c2, solve(Eq(dx.subs(t, 0), v0), c2)[0]),
    ]
)

print(f"General Solution: {x}")

# We need two specific solutions to this equation for an Trapezoid Profile:
# One that passes through (x0, v0) and has input a
# Another that passes through (xf, vf) and has input -a

# x1 is for the accelerate step
x1 = x.subs({x0: x0, v0: v0, a: a})

dx1 = x1.diff(t)
t1_eqn = solve(Eq(dx1, v), t)[0]
# x1 in phase space (input v, output x)
x1_ps = x1.subs(t, t1_eqn)


# x2 is for the decelerate step
x2 = x.subs({x0: xf, v0: vf, a: -a})

dx2 = x2.diff(t)
t2_eqn = solve(Eq(dx2, v), t)[0]
# x2 in phase space (input v, output x)
x2_ps = x2.subs(t, t2_eqn)

v_equality = Eq(x1_ps.subs(a, a1), x2_ps.subs(a, a2))
v_soln = solve(v_equality, v)


print(f"x1: {expand(simplify(x1))}")
print(f"x2: {expand(simplify(x2))}")
print(f"dx1: {expand(simplify(dx1))}")
print(f"dx2: {expand(simplify(dx2))}")
print(f"t1: {expand(simplify(t1_eqn))}")
print(f"t2: {expand(simplify(t2_eqn))}")
print(f"x1 phase space: {expand(simplify(x1.subs(t, t1_eqn)))}")
print(f"x2 phase space: {expand(simplify(x2.subs(t, t2_eqn)))}")
print(f"vi equality: {v_equality}")
print(f"v soln: {v_soln}")
