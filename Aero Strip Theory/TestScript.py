from pybrain.tools.customxml import NetworkReader
from random import uniform
from scipy.interpolate import RectBivariateSpline

import math
import numpy as np
import forcecalc

net = NetworkReader.readFrom('test2.xml')

U2Xvals = np.loadtxt('Xvals')
U2Yvals = np.loadtxt('Yvals')
U2Zvals = np.loadtxt('Zvals')
U2func = RectBivariateSpline(U2Yvals, U2Xvals, U2Zvals, s=0, kx=1, ky=1)

forcecalc.setuvals(U2Xvals, U2Yvals, U2Zvals, U2func)

max_power = 500
max_u = 20
max_v = 2
max_w = 2
max_roll = 2*math.pi
max_pitch = math.pi
max_yaw = math.pi/2
max_deflection = 30*math.pi/180


power = uniform(0, 1) * max_power
u = uniform(0, 1) * max_u
v = uniform(-1, 1) * max_v
w = uniform(-1, 1) * max_w
p = uniform(-1, 1) * max_roll
q = uniform(-1, 1) * max_pitch
r = uniform(-1, 1) * max_yaw
aileron = uniform(-1, 1) * max_deflection
elevator = uniform(-1, 1) * max_deflection
rudder = uniform(-1, 1) * max_deflection

x = uniform(0, 1)
y = uniform(0, 1)
z = uniform(0, 1)
phi = uniform(-math.pi, math.pi)
theta = uniform(-math.pi, math.pi)
psi = uniform(-math.pi, math.pi)

forces = forcecalc.forcecalc(power, u, v, w, p, q, r, aileron, elevator, rudder)
state = forcecalc.nlti(u, v, w, p, q, r, x, y, z, phi, theta, psi, forces)

u_out = state[0]
v_out = state[1]
w_out = state[2]
p_out = state[3]
q_out = state[4]
r_out = state[5]

x_out = state[6]
y_out = state[7]
z_out = state[8]
phi_out = state[9]
theta_out = state[10]
psi_out = state[11]

power /= max_power
u /= max_u
v /= max_v
w /= max_w
p /= max_roll
q /= max_pitch
r /= max_yaw

aileron /= max_deflection
elevator /= max_deflection
rudder /= max_deflection

phi /= math.pi
theta /= math.pi
psi /= math.pi

net_out = net.activate([power, u, v, w, p, q, r, -aileron, aileron, elevator, rudder, x, y, z, phi, theta, psi])
net_out[0] *= max_u
net_out[1] *= max_v
net_out[2] *= max_w
net_out[3] *= max_roll
net_out[4] *= max_pitch
net_out[5] *= max_yaw
net_out[9] *= math.pi
net_out[10] *= math.pi
net_out[11] *= math.pi

func_out = [u_out, v_out, w_out, p_out, q_out, r_out, x_out, y_out, z_out, phi_out, theta_out, psi_out]

print "U, V, W: \n {} \n {} \n".format(net_out[0:3], func_out[0:3])
print "P, Q, R: \n {} \n {} \n".format(net_out[3:6], func_out[3:6])
print "X, Y, Z: \n {} \n {} \n".format(net_out[6:9], func_out[6:9])
print "PHI, THETA, PSI: \n {} \n {} \n".format(net_out[9:12], func_out[9:12])