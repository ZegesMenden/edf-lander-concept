import matplotlib.pyplot as plt
import numpy as np
from physics import Vec3, Quat, physics_body, clamp

class EDF:

    def __init__(self):
        self.thrust = 0.0
        self.target_thrust = 0.0
        self.max_thrust = 0.0
        self.min_thrust = 0.0
        self.max_change = 100.0

    def update_target_thrust(self, thrust_target):
        self.target_thrust = thrust_target
    
    def update(self, dt):

        change = clamp(self.target_thrust, self.min_thrust, self.max_thrust)-self.thrust
        change = clamp(change, -self.max_change*dt, self.max_change*dt)
        self.thrust += change
        

    def get_current_thrust(self):
        return self.thrust
    
edf: EDF = EDF()
edf.max_thrust = 5.0*9.816
edf.min_thrust = 0.0

vehicle: physics_body = physics_body(
    # 700 ft AGL
    position=Vec3(210.0, 0.0, 0.0),
    # 80 ft/s
    velocity=Vec3(-24.0, 0.0, 0.0),
    mass=2.5,
    moment_of_inertia=Vec3(1.0, 1.0, 1.0),
    drag_coefficient_forewards=0.35,
    drag_coefficient_sideways=1.35,
    ref_area=0.035
)

def pd_calculate(err, d_err, kp, kd):
    return (err*kp) + (d_err*kd)

t_end = 15.0
time_step = 0.001
sim_time = 0.0

global burn_has_begun
burn_has_begun = False

pos_graph = []
vel_graph = []
acc_graph = []
burn_alt_graph = []

while sim_time < t_end:

    # update fan throttle calculations

    energy = vehicle.mass*( 0.5*(vehicle.velocity.length()**2) + 9.816*vehicle.position.x )
    burn_alt = energy/(edf.max_thrust*0.95)

    if vehicle.position.x < burn_alt:
        burn_has_begun = True

    if burn_has_begun == True:
        edf.update_target_thrust(energy/vehicle.position.x)
    if vehicle.position.x < 0.5:
        edf.update_target_thrust(0.0)
    
    edf.update(time_step)
    vehicle.apply_local_force(Vec3(edf.get_current_thrust(), 0.0, 0.0))
    vehicle.update(time_step)
    
    pos_graph.append(vehicle.position.x)
    vel_graph.append(vehicle.velocity.x)
    acc_graph.append(vehicle.acceleration.x)
    burn_alt_graph.append(burn_alt)
    
    vehicle.clear()

    if vehicle.position.x < 0.0:
        break

    sim_time += time_step

time_graph = np.arange(sim_time, step=time_step)

plt.plot(time_graph, pos_graph)
plt.plot(time_graph, vel_graph)
plt.plot(time_graph, acc_graph)
plt.plot(time_graph, burn_alt_graph)

plt.show()