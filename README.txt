Part 2 – Softbody Simulation System

I have completed all the required parts of the assignment 


This script turns a regular mesh into a simple softbody by treating each
vertex as a particle and connecting all particles together with springs.
The idea is that when one particle moves, the springs pull or push the
others, which makes the object look soft and flexible.


Each frame, the system applies several forces: gravity, spring forces
between particles, damping to keep the motion stable, and a collision
force if a particle goes below the ground plane. For the ground, I use
a “contact spring” that only gets created when a particle actually
penetrates the plane and disappears once the particle comes back out.

The simulation uses Symplectic Euler, which updates velocity first and
then position. It’s simple and works well enough for this kind of system.


After moving all particles, their positions are written back to the mesh
(in local space), and the normals and bounds get updated so the mesh
renders correctly. 
