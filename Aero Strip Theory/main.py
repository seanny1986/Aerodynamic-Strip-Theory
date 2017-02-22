import forcecalc
import numpy as np
import math
from scipy.interpolate import RectBivariateSpline

U2Xvals = np.loadtxt('Xvals')
U2Yvals = np.loadtxt('Yvals')
U2Zvals = np.loadtxt('Zvals')
U2func = RectBivariateSpline(U2Yvals, U2Xvals, U2Zvals, s=0, kx=1, ky=1)

timestep = 0.01
udot = 0
vdot = 0
wdot = 0
pdot = 0
qdot = 0
rdot = 0

'strip theory solver parameters'
steps = 100
rho = 1.225
g = 9.81

'########################## AIRCRAFT ############################'
'fuselage geometry'
proprad = 0.17
fuserad = 0.125
x_cg = 0.25
A_b_ref = math.pi * fuserad * fuserad
m = 2.85
Ixx = 1.5
Iyy = 1.8
Izz = 1.5

'propeller disk area'
diskA = math.pi * proprad * proprad

'fuselage drag values'
CD0_b = 0.05
dCDb_dB = 0.01
dCDb_dA = 0.01

'stall points'
alphamin = -20
alphamax = 20

'########################## WING ############################'
'wing geometry'
wspan = 1.8
winc = 3
rc_w = 0.35
tc_w = 0.11
cbar_w = (rc_w + tc_w) / 2
Sref_w = cbar_w * wspan
qtc_sweep_w = 15
wing_root_le_x = 0.45
AR_w = wspan * wspan / Sref_w
w_el = wspan / 2 / steps

'wing lift curve slope values'
dCL_da_w = 5.4
dCL_de_w = 0.6
CL0_w = 0.5
CD0_w = 0.005

'wing control surface y placement (start and end)'
Y_w = 0.65
y_w = 2.0

'oswald efficiency factor wing'
e_w = 0.8

'########################## HORIZONTAL TAIL ############################'
'horizontal tail geometry'
htspan = 0.65
htinc = 0
rc_ht = 0.25
tc_ht = 0.25
cbar_ht = (rc_ht+tc_ht)/2
Sref_ht = cbar_ht*htspan
qtc_sweep_ht = 0
htail_root_le_x = 0.95
AR_ht = htspan*htspan/Sref_ht
ht_el = htspan/2/steps

'horizontal tail lift curve slope values'
dCL_da_ht = 5.4
dCL_de_ht = 0.8
CL0_ht = 0
CD0_ht = 0.005

'horizontal tail control surface y placement (start and end)'
Y_ht = 0.1
y_ht = 0.5

'oswald efficiency factor horizontal tail'
e_ht = 0.8

'########################## VERTICAL TAIL ############################'
'vertical tail geometry'
vtspan = 0.3
vtinc = 0
rc_vt = 0.2
tc_vt = 0.2
cbar_vt = (rc_vt+tc_vt)/2
Sref_vt = cbar_vt*vtspan
qtc_sweep_vt = 15
vtail_root_le_x = 0.95
AR_vt = vtspan*vtspan/Sref_vt
vt_el = vtspan/2/steps

'vertical tail lift curve slope values'
dCL_da_vt = 5.4
dCL_de_vt = 0.8
CL0_vt = 0
CD0_vt = 0.01

'vertical tail control surface y placement (start and end)'
Y_vt = 0.1
y_vt = 0.35

e_vt = 0.8

'########################## SIMULATION PARAMETERS ############################'
max_power = 500
max_u = 20
max_v = 2
max_w = 2
max_roll = 2*math.pi
max_pitch = math.pi
max_yaw = math.pi/2
max_deflection = 30*math.pi/180

power = 0
u = 12
v = 0
w = 0
p = 0
q = 0
r = 0
aileron = 0
elevator = 0
rudder = 0

x = 0
y = 0
z = 0
phi = 0
theta = 0
psi = 0

forcecalc.setuvals(U2Xvals, U2Yvals, U2Zvals, U2func)
forcecalc.setsolverparams(timestep, udot, vdot, wdot, pdot, qdot, rdot, steps, rho, g)
forcecalc.setacparams(m, Ixx, Iyy, Izz, proprad, fuserad, x_cg, CD0_b, dCDb_dB, dCDb_dA, alphamin, alphamax)
forcecalc.setwingparams(wspan, winc, rc_w, tc_w, qtc_sweep_w, wing_root_le_x, dCL_da_w, dCL_de_w, CL0_w, CD0_w, Y_w, y_w, e_w)
forcecalc.sethtailparams(htspan, htinc, rc_ht, tc_ht, qtc_sweep_ht, htail_root_le_x, dCL_da_ht, dCL_de_ht, CL0_ht, CD0_ht, Y_ht, y_ht, e_ht)
forcecalc.setvtailparams(vtspan, vtinc, rc_vt, tc_vt, qtc_sweep_vt, vtail_root_le_x, dCL_da_vt, dCL_de_vt, CL0_vt, CD0_vt, Y_vt, y_vt, e_vt)
forcecalc.buildgeom()

forces = forcecalc.forcecalc(power, u, v, w, p, q, r, aileron, elevator, rudder)
coefficients = forcecalc.coefs(u, v, w, forces)
state = forcecalc.nlti(u, v, w, p, q, r, x, y, z, phi, theta, psi, forces)

print forces
print coefficients
print state
