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
        self.current_draw = 0.0
        self.max_current_draw = 94.0

    def update_target_thrust(self, thrust_target):
        self.target_thrust = thrust_target
    
    def update(self, dt):

        change = clamp(self.target_thrust, self.min_thrust, self.max_thrust)-self.thrust
        change = clamp(change, -self.max_change*dt, self.max_change*dt)
        self.thrust += change

        self.current_draw = clamp(self.thrust/self.max_thrust, 0.0, 1) * self.max_current_draw
        

    def get_current_thrust(self):
        return self.thrust
    
edf: EDF = EDF()
edf.max_thrust = 4*9.816
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

t_end = 13
time_step = 0.001
sim_time = 0.0

global burn_has_begun
burn_has_begun = False

global burn_alt
burn_alt = 0.0

pos_graph = []
vel_graph = []
acc_graph = []
burn_alt_graph = []
energy_graph = []
current_draw_graph = []
amp_hours_used = []
watt_hours_used = []

while sim_time < t_end:

    # update fan throttle calculations

    energy = vehicle.mass*( 0.5*(vehicle.velocity.length()**2) + 9.816*vehicle.position.x )
    
    if not burn_has_begun:
        burn_alt = energy/(edf.max_thrust*0.9)

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
    energy_graph.append(energy)
    burn_alt_graph.append(burn_alt)
    current_draw_graph.append(edf.current_draw)
    if len(amp_hours_used) > 0:
        amp_hours_used.append(amp_hours_used[-1]+(edf.current_draw/(3600/time_step)))
    else:
        amp_hours_used.append((edf.current_draw/(3600/time_step)))

    watt_hours_used.append(amp_hours_used[-1]*(12*4.2))
    vehicle.clear()

    if vehicle.position.x < 0.0:
        break

    sim_time += time_step

time_graph = np.arange(sim_time, step=time_step)

# energy_graph_metric = energy_graph

energy_graph_imperial = [energy_graph[x] * 0.737562 for x in range(len(energy_graph))]

plt.figure(1)

plt.plot(time_graph, pos_graph, label="position")
plt.plot(time_graph, vel_graph, label="velocity")
plt.plot(time_graph, acc_graph, label="acceleration")
plt.plot(time_graph, burn_alt_graph, label="engine start altitude")
plt.legend()

plt.figure(2)

plt.plot(time_graph, energy_graph, label="kinetic energy")
plt.plot(time_graph, energy_graph_imperial, label="kinetic energy imperial")
plt.legend()

plt.figure(3)

plt.plot(time_graph, acc_graph, label="EDF thrust")
plt.plot(time_graph, current_draw_graph, label="EDF current draw")
plt.plot(time_graph, amp_hours_used, label="amp hours used")
plt.plot(time_graph, watt_hours_used, label="watt hours used")
plt.legend()

plt.show()