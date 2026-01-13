import numpy as np
import matplotlib.pyplot as plt


class LiftVelocitySolver:
    def __init__(self, rho, area, cl, weight, tolerance):
        self.rho = rho
        self.area = area
        self.cl = cl
        self.weight = weight
        self.tolerance = tolerance

    # Lift - Weight equation
    def f(self, v):
        return 0.5 * self.rho * v**2 * self.area * self.cl - self.weight

    # Derivative
    def df(self, v):
        return self.rho * v * self.area * self.cl

    # Newton–Raphson Method
    def solve(self, initial_guess):
        v = initial_guess
        errors = []
        velocities = []

        while abs(self.f(v)) > self.tolerance:
            errors.append(abs(self.f(v)))
            velocities.append(v)
            v = v - self.f(v) / self.df(v)

        return v, velocities, errors


#  MAIN PROGRAM
print("__Aircraft Lift Velocity Calculator__")

rho = float(input("Enter rho (kg/m^3): "))
area = float(input("Enter wing area (m^2): "))
cl = float(input("Enter CL: "))
weight = float(input("Enter weight (N): "))
tolerance = float(input("Enter tolerance: "))
initial_guess = float(input("Enter initial guess (m/s): "))

solver = LiftVelocitySolver(rho, area, cl, weight, tolerance)

velocity, velocities, errors = solver.solve(initial_guess)

print(f"\nRequired Velocity = {velocity:.4f} m/s")

plt.plot(errors, marker = "o")
plt.xlabel("Iteration")
plt.ylabel("Error f(V)")
plt.grid(True)
plt.show()
