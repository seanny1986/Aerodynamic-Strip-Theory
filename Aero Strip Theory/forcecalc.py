from __future__ import print_function
import numpy as np
import math

global U2Xvals
global U2Yvals
global U2Zvals
global U2func

'##################--SOLVER VARIABLES--##################'
'leapfrog integrator solver vars'
global timestep
global udot
global vdot
global wdot
global pdot
global qdot
global rdot

'strip theory solver parameters'
global steps
global rho
global g


'##################--AIRCRAFT--##################'
'aircraft parameters'
global m
global Ixx
global Iyy
global Izz

'fuselage geometry'
global proprad
global fuserad
global x_cg
global A_b_ref

'propeller disk area'
global diskA

'fuselage drag values'
global CD0_b
global dCDb_dB
global dCDb_dA

'stall points'
global alphamin
global alphamax


'##################--WING--##################'
'wing geometry'
global wspan
global winc
global rc_w
global tc_w
global cbar_w
global Sref_w
global le_sweep_w
global qtc_sweep_w
global wing_root_le_x
global AR_w
global w_el
global wing
global chord_w
global x_ac_w

'wing lift curve slope values'
global dCL_da_w
global dCL_de_w
global CL0_w
global CD0_w
CM0_w = 0
dCM_da_w = 0
dCM_de_w = 0

'wing control surface y placement (start and end)'
global Y_w
global y_w

'oswald efficiency factor wing'
global e_w


'##################--HORIZONTAL TAIL--##################'
'horizontal tail geometry'
global htspan
global htinc
global rc_ht
global tc_ht
global cbar_ht
global Sref_ht
global le_sweep_ht
global qtc_sweep_ht
global htail_root_le_x
global AR_ht
global ht_el
global htail
global chord_ht
global x_ac_ht

'horizontal tail lift curve slope values'
global dCL_da_ht
global dCL_de_ht
global CL0_ht
global CD0_ht
CM0_ht = 0
dCM_da_ht = 0
dCM_de_ht = 0

'horizontal tail control surface y placement (start and end)'
global Y_ht
global y_ht

'oswald efficiency factor horizontal tail'
global e_ht


'##################--VERTICAL TAIL--##################'
'vertical tail geometry'
global vtspan
global rc_vt
global tc_vt
global cbar_vt
global Sref_vt
global le_sweep_vt
global qtc_sweep_vt
global vtail_root_le_x
global AR_vt
global vt_el
global vtail
global chord_vt
global x_ac_vt

'vertical tail lift curve slope values'
global dCL_da_vt
global dCL_de_vt
global CL0_vt
global CD0_vt
CM0_vt = 0
dCM_da_vt = 0
dCM_de_vt = 0

'vertical tail control surface y placement (start and end)'
global Y_vt
global y_vt

global e_vt

'##################--SETTERS--##################'
'set solver params'
def setsolverparams(timestep_, udot_, vdot_, wdot_, pdot_, qdot_, rdot_, steps_, rho_, g_):
    global timestep
    global udot
    global vdot
    global wdot
    global pdot
    global qdot
    global rdot

    global steps
    global rho
    global g

    timestep = timestep_
    udot = udot_
    vdot = vdot_
    wdot = wdot_
    pdot = pdot_
    qdot = qdot_
    rdot = rdot_

    'strip theory solver parameters'
    steps = steps_
    rho = rho_
    g = g_

'set u value surface parameters'
def setuvals(u2x, u2y, u2z, ufunc):
    global U2Xvals
    global U2Yvals
    global U2Zvals
    global U2func

    U2Xvals = u2x
    U2Yvals = u2y
    U2Zvals = u2z
    U2func = ufunc

'set aircraft parameters'
def setacparams(m_, Ixx_, Iyy_, Izz_, proprad_, fuserad_, x_cg_, CD0_b_, dCDb_dB_, dCDb_dA_, alphamin_, alphamax_):
    global m
    global Ixx
    global Iyy
    global Izz

    global proprad
    global fuserad
    global x_cg

    global CD0_b
    global dCDb_dB
    global dCDb_dA

    global alphamin
    global alphamax

    m = m_
    Ixx = Ixx_
    Iyy = Iyy_
    Izz = Izz_

    'fuselage geometry'
    proprad = proprad_
    fuserad = fuserad_
    x_cg = x_cg_

    'fuselage drag values'
    CD0_b = CD0_b_
    dCDb_dB = dCDb_dB_
    dCDb_dA = dCDb_dA_

    'stall points'
    alphamin = alphamin_*math.pi/180
    alphamax = alphamax_*math.pi/180

'set wing geometry'
def setwingparams(wspan_, winc_, rc_w_, tc_w_, qtc_sweep_w_, wing_root_le_x_, dCL_da_w_, dCL_de_w_, CL0_w_, CD0_w_, Y_w_, y_w_, e_w_):
    global wspan
    global winc
    global rc_w
    global tc_w
    global qtc_sweep_w
    global wing_root_le_x

    global dCL_da_w
    global dCL_de_w
    global CL0_w
    global CD0_w

    global Y_w
    global y_w

    global e_w

    wspan = wspan_
    winc = winc_*math.pi/180
    rc_w = rc_w_
    tc_w = tc_w_
    qtc_sweep_w = qtc_sweep_w_*math.pi/180
    wing_root_le_x = wing_root_le_x_

    'wing lift curve slope values'
    dCL_da_w = dCL_da_w_
    dCL_de_w = dCL_de_w_
    CL0_w = CL0_w_
    CD0_w = CD0_w_

    'wing control surface y placement (start and end)'
    Y_w = Y_w_
    y_w = y_w_

    'oswald efficiency factor wing'
    e_w = e_w_

'set horizontal tail geometry'
def sethtailparams(htspan_, htinc_, rc_ht_, tc_ht_, qtc_sweep_ht_, htail_root_le_x_, dCL_da_ht_, dCL_de_ht_, CL0_ht_, CD0_ht_, Y_ht_, y_ht_, e_ht_):
    global htspan
    global htinc
    global rc_ht
    global tc_ht
    global qtc_sweep_ht
    global htail_root_le_x

    global dCL_da_ht
    global dCL_de_ht
    global CL0_ht
    global CD0_ht

    global Y_ht
    global y_ht

    global e_ht

    htspan = htspan_
    htinc = htinc_*math.pi/180
    rc_ht = rc_ht_
    tc_ht = tc_ht_
    qtc_sweep_ht = qtc_sweep_ht_*math.pi/180
    htail_root_le_x = htail_root_le_x_

    'wing lift curve slope values'
    dCL_da_ht = dCL_da_ht_
    dCL_de_ht = dCL_de_ht_
    CL0_ht = CL0_ht_
    CD0_ht = CD0_ht_

    'wing control surface y placement (start and end)'
    Y_ht = Y_ht_
    y_ht = y_ht_

    'oswald efficiency factor wing'
    e_ht = e_ht_

'set vertical tail geometry'
def setvtailparams(vtspan_, vtinc_, rc_vt_, tc_vt_, qtc_sweep_vt_, vtail_root_le_x_, dCL_da_vt_, dCL_de_vt_, CL0_vt_, CD0_vt_, Y_vt_, y_vt_, e_vt_):
    global vtspan
    global vtinc
    global rc_vt
    global tc_vt
    global qtc_sweep_vt
    global vtail_root_le_x

    global dCL_da_vt
    global dCL_de_vt
    global CL0_vt
    global CD0_vt

    global Y_vt
    global y_vt

    global e_vt

    vtspan = vtspan_
    vtinc = vtinc_*math.pi/180
    rc_vt = rc_vt_
    tc_vt = tc_vt_
    qtc_sweep_vt = qtc_sweep_vt_*math.pi/180
    vtail_root_le_x = vtail_root_le_x_

    'wing lift curve slope values'
    dCL_da_vt = dCL_da_vt_
    dCL_de_vt = dCL_de_vt_
    CL0_vt = CL0_vt_
    CD0_vt = CD0_vt_

    'wing control surface y placement (start and end)'
    Y_vt = Y_vt_
    y_vt = y_vt_

    'oswald efficiency factor wing'
    e_vt = e_vt_

'build wing geometry'
def buildwing():
    global wing
    global qtc_sweep_w
    global le_sweep_w
    global chord_w
    global x_ac_w
    global AR_w
    global w_el
    global Sref_w

    b_w = wspan/2
    cbar_w = (rc_w+tc_w)/2
    Sref_w = cbar_w*wspan
    AR_w = wspan*wspan/Sref_w
    w_el = b_w/steps
    wing = np.linspace(w_el/2, (wspan-w_el)/2, steps)
    chord_w = chord(wing, wspan, Sref_w, rc_w, tc_w)
    le_sweep_w = math.atan2((b_w*math.tan(qtc_sweep_w)+0.25*(rc_w-tc_w)), b_w)
    x_ac_w = wing_root_le_x+0.25*chord_w+np.multiply(math.tan(le_sweep_w), wing)

def buildhoztail():
    global htail
    global qtc_sweep_ht
    global le_sweep_ht
    global chord_ht
    global x_ac_ht
    global AR_ht
    global ht_el
    global Sref_ht

    b_ht = htspan / 2
    cbar_ht = (rc_ht + tc_ht) / 2
    Sref_ht = cbar_ht * htspan
    AR_ht = htspan * htspan / Sref_ht
    ht_el = b_ht / steps
    htail = np.linspace(ht_el / 2, (htspan - ht_el) / 2, steps)
    chord_ht = chord(htail, htspan, Sref_ht, rc_ht, tc_ht)
    le_sweep_ht = math.atan2((b_ht * math.tan(qtc_sweep_ht) + 0.25 * (rc_ht - tc_ht)), b_ht)
    x_ac_ht = htail_root_le_x + 0.25 * chord_ht + np.multiply(math.tan(le_sweep_ht), htail)

def buildvertail():
    global vtail
    global qtc_sweep_vt
    global le_sweep_vt
    global chord_vt
    global x_ac_vt
    global AR_vt
    global vt_el
    global Sref_vt

    cbar_vt = (rc_vt + tc_vt) / 2
    Sref_vt = cbar_vt * vtspan
    AR_vt = vtspan * vtspan / Sref_vt
    vt_el = vtspan / steps
    vtail = np.linspace(vt_el / 2, (vtspan - vt_el) / 2, steps)
    chord_vt = chord(vtail, vtspan, Sref_vt, rc_vt, tc_vt)
    le_sweep_vt = math.atan2((vtspan * math.tan(qtc_sweep_vt) + 0.25 * (rc_vt - tc_vt)), vtspan)
    x_ac_vt = vtail_root_le_x + 0.25 * chord_vt + np.multiply(math.tan(le_sweep_vt), vtail)

def buildfuseandprop():
    global A_b_ref
    global diskA

    A_b_ref = math.pi * fuserad * fuserad
    diskA = math.pi * proprad * proprad

'build aircraft geometry to be used for forcecalc'
def buildgeom():
    global wing
    global htail
    global vtail
    global qtc_sweep_w
    global qtc_sweep_ht
    global qtc_sweep_vt
    global le_sweep_w
    global le_sweep_ht
    global le_sweep_vt
    global chord_w
    global chord_ht
    global chord_vt
    global x_ac_w
    global x_ac_ht
    global x_ac_vt
    global AR_w
    global AR_ht
    global AR_vt
    global w_el
    global ht_el
    global vt_el
    global A_b_ref
    global diskA
    global Sref_w
    global Sref_ht
    global Sref_vt
    global cbar_w
    global cbar_ht
    global cbar_vt
    global dCL_da_w
    global dCL_da_ht
    global dCL_da_vt

    lambda_w = tc_w/rc_w
    lambda_ht = tc_ht/rc_ht
    lambda_vt = tc_vt / rc_vt

    b_w = wspan/2
    b_ht = htspan/2

    cbar_w = rc_w*(2.0/3.0)*(1+lambda_w+lambda_w**2)/(1+lambda_w)
    Sref_w = ((rc_w+tc_w)/2)*wspan
    AR_w = wspan*wspan/Sref_w
    w_el = b_w/steps

    cbar_ht = rc_ht*(2.0/3.0)*(1+lambda_ht+lambda_ht**2)/(1+lambda_ht)
    Sref_ht = ((rc_ht+tc_ht)/2)*htspan
    AR_ht = htspan*htspan/Sref_ht
    ht_el = b_ht/steps

    cbar_vt = rc_vt*(2.0/3.0)*(1+lambda_vt+lambda_vt**2)/(1+lambda_vt)
    Sref_vt = ((rc_vt+tc_vt)/2)*vtspan
    AR_vt = vtspan*vtspan/Sref_vt
    vt_el = vtspan/steps

    wing = np.linspace(w_el/2, (wspan-w_el)/2, steps)
    htail = np.linspace(ht_el/2, (htspan-ht_el)/2, steps)
    vtail = np.linspace(vt_el/2, (vtspan-vt_el)/2, steps)

    chord_w = chord(wing, wspan, Sref_w, rc_w, tc_w)
    chord_ht = chord(htail, htspan, Sref_ht, rc_ht, tc_ht)
    chord_vt = chord(vtail, vtspan, Sref_vt, rc_vt, tc_vt)

    le_sweep_w = math.atan2((b_w*math.tan(qtc_sweep_w)+0.25*(rc_w-tc_w)), b_w)
    le_sweep_ht = math.atan2((b_ht*math.tan(qtc_sweep_ht)+0.25*(rc_ht-tc_ht)), b_ht)
    le_sweep_vt = math.atan2((vtspan*math.tan(qtc_sweep_vt)+0.25*(rc_vt-tc_vt)), vtspan)

    x_ac_w = wing_root_le_x+0.25*chord_w+np.multiply(math.tan(le_sweep_w), wing)
    x_ac_ht = htail_root_le_x+0.25*chord_ht+np.multiply(math.tan(le_sweep_ht), htail)
    x_ac_vt = vtail_root_le_x+0.25*chord_vt+np.multiply(math.tan(le_sweep_vt), vtail)

    A_b_ref = math.pi*fuserad*fuserad
    diskA = math.pi*proprad*proprad

    dCL_da_w = dCL_da_w*AR_w/(2+AR_w)
    dCL_da_ht = dCL_da_ht*AR_ht/(2 + AR_ht)
    dCL_da_vt = dCL_da_vt*AR_vt/(2 + AR_vt)


def forcecalc(power_, u_, v_, w_, p_, q_, r_, aileron_, elevator_, rudder_):

    thrust = thrustcalc(power_, u_)

    u_w_lw = u_-wing*r_
    u_w_rw = u_+wing*r_
    v_w = v_*np.ones(np.size(wing))
    w_w_lw = w_-p_*wing-q_*(x_cg-x_ac_w)
    w_w_rw = w_+p_*wing-q_*(x_cg-x_ac_w)

    u_ht_lht = u_-htail*r_
    u_ht_rht = u_+htail*r_
    v_ht = v_*np.ones(np.size(wing))
    w_ht_lht = w_-p_*htail-q_*(x_cg-x_ac_ht)
    w_ht_rht = w_+p_*htail-q_*(x_cg-x_ac_ht)

    u_vt = u_-vtail*q_
    v_vt = v_+p_*vtail+r_*(x_cg-x_ac_vt)
    w_vt = w_*np.ones(np.size(vtail))

    alpha_lw = np.arctan2(w_w_lw, u_w_lw)
    alpha_rw = np.arctan2(w_w_rw, u_w_rw)
    alpha_lht = np.arctan2(w_ht_lht, u_ht_lht)
    alpha_rht = np.arctan2(w_ht_rht, u_ht_rht)
    alpha_vt = np.arcsin(v_vt/np.sqrt(u_vt**2+v_vt**2+w_vt**2))

    CL_lw = CL(wing, dCL_da_w, alpha_lw, CL0_w, -aileron_, dCL_de_w, Y_w, y_w)
    CL_rw = CL(wing, dCL_da_w, alpha_rw, CL0_w, aileron_, dCL_de_w, Y_w, y_w)
    CL_lht = CL(htail, dCL_da_ht, alpha_lht, CL0_ht, elevator_, dCL_de_ht, Y_ht, y_ht)
    CL_rht = CL(htail, dCL_da_ht, alpha_rht, CL0_ht, elevator_, dCL_de_ht, Y_ht, y_ht)
    CL_vt = CL(vtail, dCL_da_vt, alpha_vt, CL0_vt, rudder_, dCL_de_vt, Y_vt, y_vt)

    CM_lw = CM(wing, dCM_da_w, alpha_lw, CM0_w, -aileron_, dCM_de_w, Y_w, y_w)
    CM_rw = CM(wing, dCM_da_w, alpha_lw, CM0_w, aileron_, dCM_de_w, Y_w, y_w)
    CM_lht = CM(htail, dCM_da_ht, alpha_lw, CM0_ht, -aileron_, dCM_de_w, Y_w, y_w)
    CM_rht = CM(htail, dCM_da_ht, alpha_lw, CM0_ht, -aileron_, dCM_de_w, Y_w, y_w)
    CM_vt = CM(vtail, dCM_da_vt, alpha_lw, CM0_vt, -aileron_, dCM_de_w, Y_w, y_w)

    AR_w = wspan**2/Sref_w
    AR_ht = htspan**2/Sref_ht
    AR_vt = vtspan**2/Sref_vt

    K1 = AR_w*e_w*math.pi
    K2 = AR_ht*e_ht*math.pi
    K3 = AR_vt*e_vt*math.pi

    CD_lw = CD0_w+CL_lw**2/K1
    CD_rw = CD0_w+CL_rw**2/K1
    CD_lht = CD0_ht+CL_lht**2/K2
    CD_rht = CD0_ht+CL_rht**2/K2
    CD_vt = CD0_vt+CL_vt**2/K3

    Vsq_lw = u_w_lw**2+v_w**2+w_w_lw**2
    Vsq_rw = u_w_rw**2+v_w**2+w_w_rw**2
    Vsq_lht = u_ht_lht**2+v_ht**2+w_ht_lht**2
    Vsq_rht = u_ht_rht**2+v_ht**2+w_ht_rht**2
    Vsq_vt = u_vt**2+v_vt**2+w_vt**2

    K = 0.5*rho
    A_w = w_el*chord_w
    A_ht = ht_el*chord_ht
    A_vt = vt_el*chord_vt

    LIFT_LW = CL_lw*K*Vsq_lw*A_w
    LIFT_RW = CL_rw*K*Vsq_rw*A_w
    LIFT_LHT = CL_lht*K*Vsq_lht*A_ht
    LIFT_RHT = CL_rht*K*Vsq_rht*A_ht
    LIFT_VT = CL_vt*K*Vsq_vt*A_vt

    DRAG_LW = CD_lw*K*Vsq_lw*A_w
    DRAG_RW = CD_rw*K*Vsq_rw*A_w
    DRAG_LHT = CD_lht*K*Vsq_lht*A_ht
    DRAG_RHT = CD_rht*K*Vsq_rht*A_ht
    DRAG_VT = CD_vt*K*Vsq_vt*A_vt

    PITCH_LW = CM_lw*K*Vsq_lw*A_w*chord_w
    PITCH_RW = CM_rw*K*Vsq_rw*A_w*chord_w
    PITCH_LHT = CM_lht*K*Vsq_lht*A_ht*chord_ht
    PITCH_RHT = CM_rht*K*Vsq_rht*A_ht*chord_ht
    PITCH_VT = CM_vt*K*Vsq_vt*A_vt*chord_vt

    TOTAL_PITCH = PITCH_LW+PITCH_RW+PITCH_LHT+PITCH_RHT+PITCH_VT

    LW_X = LIFT_LW*np.sin(alpha_lw)-DRAG_LW*np.cos(alpha_lw)
    RW_X = LIFT_RW*np.sin(alpha_rw)-DRAG_RW*np.cos(alpha_rw)
    LHT_X = LIFT_LHT*np.sin(alpha_lht)-DRAG_LHT*np.cos(alpha_lht)
    RHT_X = LIFT_RHT*np.sin(alpha_rht)-DRAG_RHT*np.cos(alpha_rht)
    VT_X = LIFT_VT*np.sin(alpha_vt)-DRAG_VT*np.cos(alpha_vt)

    VT_Y = LIFT_VT*np.cos(alpha_vt)+DRAG_VT*np.sin(alpha_vt)

    LW_Z = LIFT_LW*np.cos(alpha_lw)+DRAG_LW*np.sin(alpha_lw)
    RW_Z = LIFT_RW*np.cos(alpha_rw)+DRAG_RW*np.sin(alpha_rw)
    LHT_Z = LIFT_LHT*np.cos(alpha_lht)+DRAG_LHT*np.sin(alpha_lht)
    RHT_Z = LIFT_RHT*np.cos(alpha_rht)+DRAG_RHT*np.sin(alpha_rht)


    'Forces'
    XF = float(thrust)-np.sum(LW_X)-np.sum(RW_X)-np.sum(LHT_X)-np.sum(RHT_X)-np.sum(VT_X)+0.32*0.5*rho*(u_**2+v_**2)*math.pi*fuserad*fuserad
    YF = np.sum(VT_Y)
    ZF = np.sum(LW_Z)+np.sum(RW_Z)+np.sum(LHT_Z)+np.sum(RHT_Z)

    test = np.sum((RW_X-LW_X)*wing)
    test2 = np.sum((RHT_X-LHT_X)*htail)

    'Moments'
    LM = np.sum(wing*LW_Z)-np.sum(wing*RW_Z)+np.sum(htail*LHT_Z)-np.sum(htail*RHT_Z)+np.sum(vtail*VT_Y)
    MM = np.sum(LW_Z*(x_cg-x_ac_w))+np.sum(RW_Z*(x_cg-x_ac_w))+np.sum(LHT_Z*(x_cg-x_ac_ht))+np.sum(RHT_Z*(x_cg-x_ac_ht))+np.sum(vtail*VT_X)+np.sum(TOTAL_PITCH)
    NM = np.sum((RW_X-LW_X)*wing)+np.sum((RHT_X-LHT_X)*htail)

    return [XF, YF, ZF, LM, MM, NM]


def thrustcalc(power, u_):
    if power>0:
        u2 = U2func(power, u_)
        force = 0.5*rho*diskA*(u2**2-u_**2)
    else:
        force = 0
    return force


def chord(wing, span, area, rc, tc):
    k = tc/rc
    A = 2*area/((1+k)*span)
    B = 1*(1-k)/span
    res = A*(1-B*wing)
    return res


def CL(wing, dCL_da, alpha, CL0, displacement, dCL_de, pos1, pos2):
    aileronCL = heaviside(wing, pos1, pos2)
    stalled = (alpha >= alphamin) & (alpha <= alphamax)
    res = stalled.astype(int)*(CL0+dCL_da*alpha+aileronCL*dCL_de*displacement)
    return res

def CM(wing, dCM_da, alpha, CM0, displacement, dCM_de, pos1, pos2):
    aileronCL = heaviside(wing, pos1, pos2)
    stalled = (alpha >= alphamin) & (alpha <= alphamax)
    res = stalled.astype(int)*(CM0+dCM_da*alpha+aileronCL*dCM_de*displacement)
    return res


def heaviside(wing, pos1, pos2):
    res = (wing >= pos1) & (wing <= pos2)
    return res.astype(int)


def nlti(u_, v_, w_, p_, q_, r_, x_, y_, z_, phi_, theta_, psi_, A_):
    global udot
    global vdot
    global wdot
    global pdot
    global qdot
    global rdot

    du_dt = float(A_[0]/m-g*math.sin(theta_)-q_*w_+r_*v_)
    dv_dt = float(A_[1]/m+g*math.cos(theta_)*math.sin(phi_)-r_*u_+p_*w_)
    dw_dt = float(A_[2]/m+g*math.cos(theta_)*math.cos(phi_)-p_*v_+q_*u_)

    dp_dt = float(A_[3]/Ixx-(Izz-Iyy)/Ixx*q_*r_)
    dq_dt = float(A_[4]/Iyy-(Ixx - Izz)/Iyy*r_*p_)
    dr_dt = float(A_[5]/Izz-(Iyy - Ixx)/Izz*p_*q_)

    u_ += 0.5*(udot+du_dt)*timestep
    v_ += 0.5*(vdot+dv_dt)*timestep
    w_ += 0.5*(wdot+dw_dt)*timestep

    p_ += 0.5*(pdot+dp_dt)*timestep
    q_ += 0.5*(qdot+dq_dt)*timestep
    r_ += 0.5*(rdot+dr_dt)*timestep

    I_ = lindcm([phi_, theta_, psi_], [du_dt, dv_dt, dw_dt])
    X_ = lindcm([phi_, theta_, psi_], [u_, v_, w_])
    J_ = angdcm([phi_, theta_, psi_], [dp_dt, dq_dt, dr_dt])
    W_ = angdcm([phi_, theta_, psi_], [p_, q_, r_])

    dx_dt = X_[0]
    dy_dt = X_[1]
    dz_dt = X_[2]
    dphi_dt = W_[0]
    dtheta_dt = W_[1]
    dpsi_dt = W_[2]

    x_ += dx_dt*timestep+0.5*I_[0]*timestep*timestep
    y_ += dy_dt*timestep+0.5*I_[1]*timestep*timestep
    z_ += dz_dt*timestep+0.5*I_[2]*timestep*timestep

    phi_ += dphi_dt*timestep+0.5*J_[0]*timestep*timestep
    theta_ += dtheta_dt*timestep+0.5*J_[1]*timestep*timestep
    psi_ += dpsi_dt*timestep+0.5*J_[2]*timestep*timestep

    udot = du_dt
    vdot = dv_dt
    wdot = dw_dt
    pdot = dp_dt
    qdot = dq_dt
    rdot = dr_dt

    return [u_, v_, w_, p_, q_, r_, x_, y_, z_, phi_, theta_, psi_]


def lindcm(A, B):
    phiv = A[0]
    thetav = A[1]
    psiv = A[2]

    roll = np.array([[1, 0, 0],
                     [0, math.cos(phiv), math.sin(phiv)],
                     [0, -math.sin(phiv), math.cos(phiv)]])
    pitch = np.array([[math.cos(thetav), 0, -math.sin(thetav)],
                      [0, 1, 0],
                      [math.sin(thetav), 0, math.cos(thetav)]])
    yaw = np.array([[math.cos(psiv), math.sin(psiv), 0],
                    [-math.sin(psiv), math.cos(psiv), 0],
                    [0, 0, 1]])

    temp = np.matmul(roll, pitch)
    DCM = np.matmul(temp, yaw)
    Xv = np.dot(DCM, np.array(B))
    return Xv


def angdcm(A, B):
    phiv = A[0]
    thetav = A[1]

    DCM = np.array([[1, math.sin(phiv)*math.tan(thetav), math.cos(phiv)*math.tan(thetav)],
                    [0, math.cos(phiv), -math.sin(phiv)],
                    [0, math.sin(phiv)/math.cos(thetav), math.cos(phiv)/math.cos(thetav)]])

    W = np.dot(DCM, np.array(B))
    return W

def coefs(u_, v_, w_, A_):
    XF = A_[0]
    YF = A_[1]
    ZF = A_[2]
    LM = A_[3]
    MM = A_[4]
    NM = A_[5]

    q = 0.5*rho*(u_**2+v_**2+w_**2)

    CX = XF/q/Sref_w
    CY = YF/q/Sref_w
    CZ = ZF/q/Sref_w
    CL = LM/q/Sref_w/wspan
    CM = MM/q/Sref_w/cbar_w
    CN = NM/q/Sref_w/wspan

    return [CX, CY, CZ, CL, CM, CN]

def plotwing():
    'something goes here'