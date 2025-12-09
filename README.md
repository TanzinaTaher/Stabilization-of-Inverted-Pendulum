# Stabilization-of-Inverted-Pendulum


This project investigates the modeling and control of the Quanser IP02 inverted pendulum, a classical underactuated and open-loop unstable mechanical system. Starting from the nonlinear dynamics and official Quanser parameter sets (long and short pendulum), the system was linearized around the upright equilibrium and represented as both a full four-state model and a reduced pendulum-only model.

Three controllers were designed and evaluated:

a classical PID controller (on the reduced model),

full-state Pole Placement, and

optimal LQR control.

Each controller was simulated from identical initial conditions and evaluated in terms of stabilization performance, transient behavior, cart motion, and required control effort.

The results show that while PID is effective only on the reduced model, Pole Placement achieves very fast stabilization but requires unrealistically large control forces. LQR provides the most balanced and practical behavior, achieving stable regulation with smooth control action and significantly lower actuator demand. 
