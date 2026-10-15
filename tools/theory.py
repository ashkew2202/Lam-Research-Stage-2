# Numbers for the hand calculations in Q1.2 (velocity, Reynolds number) and Q3.1 (residence time)
#
#   python3 tools/theory.py
#
# v = Q/A,  Re = rho*v*d/mu
# pressure drop per metre = f*rho*v^2/(2d) with the Blasius friction factor f = 0.316*Re^-0.25

import math

RHO = 1000.0  # kg/m3
MU = 1e-3     # Pa.s


def velocity(q_lpm, d_mm):
    q = q_lpm / 60000.0  # m3/s
    d = d_mm / 1000.0
    area = math.pi * d * d / 4
    v = q / area
    re = RHO * v * d / MU
    return v, re


def dp_per_m(q_lpm, d_mm):
    v, re = velocity(q_lpm, d_mm)
    f = 64 / re if re < 2300 else 0.316 * re ** -0.25
    return f * RHO * v * v / (2 * d_mm / 1000.0)


print("Q1.2 theory, 8 mm diameter as given in the paper")
print("Q (L/min)   v (m/s)   Re")
for q in (1, 3, 6):
    v, re = velocity(q, 8)
    print(f"{q:<10}  {v:.3f}     {re:.0f}")

print()
print("Actual tube is 8 mm outside, 5 mm bore. Area ratio (8/5)^2 =", round((8 / 5) ** 2, 2))
print("Q (L/min)   v at 8 mm   v at 5 mm   Re at 5 mm   dp per m (kPa)")
for q in (1.20, 2.25, 2.30, 3, 6):
    v8, _ = velocity(q, 8)
    v5, re5 = velocity(q, 5)
    print(f"{q:<10}  {v8:.2f}        {v5:.2f}        {re5:<10.0f}   {dp_per_m(q, 5) / 1000:.0f}")

# flow where Re = 2300 in the 5 mm bore
v_lam = 2300 * MU / (RHO * 0.005)
print(f"laminar limit (Re 2300) in the 5 mm bore is at {v_lam * math.pi * 0.005 ** 2 / 4 * 60000:.2f} L/min")

print()
print("Q3.1 residence time = V/Q")
for vol, q in ((0.5, 2.5), (1.0, 2.5), (0.5, 5.0)):
    print(f"V = {vol} L, Q = {q} L/min:  {vol / q:.2f} min = {vol / q * 60:.0f} s")
