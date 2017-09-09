from __future__ import print_function
import numpy as np

class forcecalc(object):
    
    def __init__(self):
        pass

    ##################--SETTERS--##################
    def setsolverparams(self, 
                        timestep, 
                        udot, 
                        vdot, 
                        wdot, 
                        pdot, 
                        qdot, 
                        rdot, 
                        steps, 
                        rho, 
                        g):
        # leapfrog integrator solver vars 
        self.timestep_ = timestep
        self.udot_ = udot
        self.vdot_ = vdot
        self.wdot_ = wdot
        self.pdot_ = pdot
        self.qdot_ = qdot
        self.rdot_ = rdot

        # strip theory solver parameters
        self.steps_ = steps
        self.rho_ = rho
        self.g_ = g

        # set u value surface parameters
    def setuvals(   self, 
                    U2Xvals, 
                    U2Yvals, 
                    U2Zvals, 
                    U2func):
        self.U2Xvals_ = U2Xvals
        self.U2Yvals_ = U2Yvals
        self.U2Zvals_ = U2Zvals
        self.U2func_ = U2func

    # set aircraft parameters
    def setacparams(self, 
                    m, 
                    Ixx, 
                    Iyy, 
                    Izz, 
                    proprad, 
                    fuserad, 
                    x_cg, 
                    CD0_b, 
                    dCDb_dB, 
                    dCDb_dA, 
                    alphamin, 
                    alphamax):
        self.m_ = m
        self.Ixx_ = Ixx
        self.Iyy_ = Iyy
        self.Izz_ = Izz

        # fuselage geometry
        self.proprad_ = proprad
        self.fuserad_ = fuserad
        self.x_cg_ = x_cg

        # fuselage drag values
        self.CD0_b_ = CD0_b
        self.dCDb_dB_ = dCDb_dB
        self.dCDb_dA_ = dCDb_dA

        # stall points
        self.alphamin_ = alphamin*np.pi/180
        self.alphamax_ = alphamax*np.pi/180

    # set wing geometry
    def setwingparams(  self, 
                        wspan, 
                        winc, 
                        rc_w, 
                        tc_w, 
                        qtc_sweep_w, 
                        wing_root_le_x, 
                        dCL_da_w, 
                        dCL_de_w, 
                        CL0_w, 
                        CD0_w, 
                        Y_w, 
                        y_w, 
                        e_w):
        self.wspan_ = wspan
        self.winc_ = winc*np.pi/180
        self.rc_w_ = rc_w
        self.tc_w_ = tc_w
        self.qtc_sweep_w_ = qtc_sweep_w*np.pi/180
        self.wing_root_le_x_ = wing_root_le_x

        # wing lift curve slope values
        self.dCL_da_w_ = dCL_da_w
        self.dCL_de_w_ = dCL_de_w
        self.CL0_w_ = CL0_w
        self.CD0_w_ = CD0_w

        # wing control surface y placement (start and end)'
        self.Y_w_ = Y_w
        self.y_w_ = y_w

        # oswald efficiency factor wing'
        self.e_w_ = e_w

    # set horizontal tail geometry
    def sethtailparams( self, 
                        htspan, 
                        htinc, 
                        rc_ht, 
                        tc_ht, 
                        qtc_sweep_ht, 
                        htail_root_le_x, 
                        dCL_da_ht, 
                        dCL_de_ht, 
                        CL0_ht, 
                        CD0_ht, 
                        Y_ht, 
                        y_ht, 
                        e_ht):
        self.htspan_ = htspan
        self.htinc_ = htinc*np.pi/180
        self.rc_ht_ = rc_ht
        self.tc_ht_ = tc_ht
        self.qtc_sweep_ht_ = qtc_sweep_ht*np.pi/180
        self.htail_root_le_x_ = htail_root_le_x

        # horizontal tailplane lift-curve slope values
        self.dCL_da_ht_ = dCL_da_ht
        self.dCL_de_ht_ = dCL_de_ht
        self.CL0_ht_ = CL0_ht
        self.CD0_ht_ = CD0_ht

        # htail control surface y placement (start and end)
        self.Y_ht_ = Y_ht
        self.y_ht_ = y_ht

        # oswald efficiency factor htail
        self.e_ht_ = e_ht

    # set vertical tail geometry
    def setvtailparams( self, 
                        vtspan, 
                        vtinc, 
                        rc_vt, 
                        tc_vt, 
                        qtc_sweep_vt, 
                        vtail_root_le_x, 
                        dCL_da_vt, 
                        dCL_de_vt, 
                        CL0_vt, 
                        CD0_vt, 
                        Y_vt, 
                        y_vt, 
                        e_vt):
        self.vtspan_ = vtspan
        self.vtinc_ = vtinc*np.pi/180
        self.rc_vt_ = rc_vt
        self.tc_vt_ = tc_vt
        self.qtc_sweep_vt_ = qtc_sweep_vt*np.pi/180
        self.vtail_root_le_x_ = vtail_root_le_x

        # wing lift curve slope values
        self.dCL_da_vt_ = dCL_da_vt
        self.dCL_de_vt_ = dCL_de_vt
        self.CL0_vt_ = CL0_vt
        self.CD0_vt_ = CD0_vt

        # wing control surface y placement (start and end)
        self.Y_vt_ = Y_vt
        self.y_vt_ = y_vt

        # oswald efficiency factor wing
        self.e_vt_ = e_vt

    #build wing geometry
    def buildwing(self):
        span = self.wspan_
        rc = self.rc_w_
        tc = self.tc_w_
        steps = self.steps_
        rootx = self.wing_root_le_x_

        self.b_w_ = span/2
        self.cbar_w_ = (rc+tc)/2
        self.Sref_w_ = self.cbar_w_*span
        self.AR_w_ = span**2/self.Sref_w_
        self.w_el_ = self.b_w_/steps
        self.wing_ = np.linspace(self.w_el_/2, (span-self.w_el_)/2, steps)
        self.chord_w_ = self.chord(self.wing_, span, self.Sref_w_, rc, tc)
        self.le_sweep_w_ = np.arctan2((self.b_w_*np.tan(self.qtc_sweep_w_)+0.25*(rc-tc)), self.b_w_)
        self.x_ac_w_ = rootx+0.25*self.chord_w_+np.multiply(np.tan(self.le_sweep_w_), self.wing_)

    # build horizontal tail geometry
    def buildhoztail(self):
        span = self.htspan_
        rc = self.rc_ht_
        tc = self.tc_ht_
        steps = self.steps_
        rootx = self.htail_root_le_x_

        self.b_ht_ = span/2
        self.cbar_ht_ = (rc+tc)/2
        self.Sref_ht_ = self.cbar_ht_*span
        self.AR_ht_ = span**2/self.Sref_ht_
        self.ht_el_ = self.b_ht_/steps
        self.htail_ = np.linspace(self.ht_el_/2, (span-self.ht_el_)/2, steps)
        self.chord_ht_ = self.chord(self.htail_, span, self.Sref_ht_, rc, tc)
        self.le_sweep_ht_ = np.arctan2((self.b_ht_*np.tan(self.qtc_sweep_ht_)+0.25*(rc-tc)), self.b_ht_)
        self.x_ac_ht_ = rootx+0.25*self.chord_ht_+np.multiply(np.tan(self.le_sweep_ht_), self.htail_)

    # build vertical tail geometry
    def buildvertail(self):
        span = self.vtspan_
        rc = self.rc_vt_
        tc = self.tc_vt_
        steps = self.steps_
        rootx = self.vtail_root_le_x_

        self.cbar_vt_ = (rc+tc)/2
        self.Sref_vt_ = self.cbar_vt_*span
        self.AR_vt_ = span**2/self.Sref_vt_
        self.vt_el_ = span/steps
        self.vtail_ = np.linspace(self.vt_el_/2, (span-self.vt_el_)/2, steps)
        self.chord_vt_ = self.chord(self.vtail_, span, self.Sref_vt_, rc, tc)
        self.le_sweep_vt_ = np.arctan2((span*np.tan(self.qtc_sweep_vt_)+0.25*(rc-tc)), span)
        self.x_ac_vt_ = rootx+0.25*self.chord_vt_+np.multiply(np.tan(self.le_sweep_vt_), self.vtail_)

    # build fuselage and prop geometry
    def buildfuseandprop(self):
        self.A_b_ref_ = np.pi*self.fuserad_**2
        self.diskA_ = np.pi*self.proprad_**2

    # build aircraft geometry to be used for forcecalc
    def buildgeom(self):
        self.buildwing()
        self.buildhoztail()
        self.buildvertail()
        self.buildfuseandprop()

    # calculate body forces acting on the aircraft using strip theory
    def forcecalc(  self, 
                    power, 
                    u, 
                    v, 
                    w, 
                    p, 
                    q, 
                    r, 
                    aileron, 
                    elevator, 
                    rudder):
        # calc thrust force
        thrust = self.thrustcalc(power, u)
    
        # creating left and right wings to keep axes consistent
        lw = -np.flip(self.wing_, 0)
        rw = self.wing_

        # calc local velocity components for each strip on the wing (u,v,w)
        u_w_lw = u+lw*r
        u_w_rw = u+rw*r
        v_w = v*np.ones(np.size(rw))
        w_w_lw = w+p*lw-q*(self.x_cg_-self.x_ac_w_)
        w_w_rw = w+p*rw-q*(self.x_cg_-self.x_ac_w_)

        # calc local velocity components for each strip on the horizontal tail (u,v,w)'
        lht = -np.flip(self.htail_, 0)
        rht = self.htail_
        u_ht_lht = u+lht*r
        u_ht_rht = u+rht*r
        v_ht = v-r*(self.x_cg_-self.x_ac_ht_)
        w_ht_lht = w+p*lht-q*(self.x_cg_-self.x_ac_ht_)
        w_ht_rht = w+p*rht-q*(self.x_cg_-self.x_ac_ht_)
    
        # calc local velocity components for each strip on the vertical tail (u,v,w)
        u_vt = u-self.vtail_*q
        v_vt = v+p*self.vtail_-r*(self.x_cg_-self.x_ac_vt_)
        w_vt = w-q*(self.x_cg_-self.x_ac_vt_)
    
        # calc local local angles of attack for each strip on the wings, ht, vt, including wing incidence
        alpha_lw = np.arctan2(w_w_lw, u_w_lw)+self.winc_*np.pi/180
        alpha_rw = np.arctan2(w_w_rw, u_w_rw)+self.winc_*np.pi/180
        alpha_lht = np.arctan2(w_ht_lht, u_ht_lht)+self.htinc_*np.pi/180
        alpha_rht = np.arctan2(w_ht_rht, u_ht_rht)+self.htinc_*np.pi/180
        alpha_vt = np.arcsin(v_vt/np.sqrt(u_vt**2+v_vt**2+w_vt**2))
    
        # calc local local lift coefficients for each strip on the wings, ht, vt
        CL_lw = self.CL(lw, self.dCL_da_w_, alpha_lw, self.CL0_w_, -aileron, self.dCL_de_w_, -self.Y_w_, -self.y_w_)
        CL_rw = self.CL(rw, self.dCL_da_w_, alpha_rw, self.CL0_w_, aileron, self.dCL_de_w_, self.Y_w_, self.y_w_)
        CL_lht = self.CL(lht, self.dCL_da_ht_, alpha_lht, self.CL0_ht_, elevator, self.dCL_de_ht_, self.Y_ht_, self.y_ht_)
        CL_rht = self.CL(rht, self.dCL_da_ht_, alpha_rht, self.CL0_ht_, elevator, self.dCL_de_ht_, self.Y_ht_, self.y_ht_)
        CL_vt = self.CL(self.vtail_, self.dCL_da_vt_, alpha_vt, self.CL0_vt_, rudder, self.dCL_de_vt_, self.Y_vt_, self.y_vt_)
    
        # calc local local moment coefficients for each strip on the wings, ht, vt
        #CM_lw = self.CM(lw, self.dCM_da_w_, alpha_lw, self.CM0_w_, -aileron, self.dCM_de_w_, self.Y_w_, self.y_w_)
        #CM_rw = self.CM(rw, self.dCM_da_w_, alpha_lw, self.CM0_w_, aileron, self.dCM_de_w_, self.Y_w_, self.y_w_)
        #CM_lht = self.CM(lht, self.dCM_da_ht_, alpha_lw, self.CM0_ht_, elevator, self.dCM_de_w_, self.Y_w_, self.y_w_)
        #CM_rht = self.CM(rht, self.dCM_da_ht_, alpha_lw, self.CM0_ht_, elevator, self.dCM_de_w_, self.Y_w_, self.y_w_)
        #CM_vt = self.CM(self.vtail_, self.dCM_da_vt_, alpha_lw, self.CM0_vt_, rudder, self.dCM_de_w_, self.Y_w_, self.y_w_)
    
        # calc constant values
        K1 = self.AR_w_*self.e_w_*np.pi
        K2 = self.AR_ht_*self.e_ht_*np.pi
        K3 = self.AR_vt_*self.e_vt_*np.pi
    
        # calc drag coefficients for wings, ht, vt
        CD_lw = self.CD0_w_+CL_lw**2/K1
        CD_rw = self.CD0_w_+CL_rw**2/K1
        CD_lht = self.CD0_ht_+CL_lht**2/K2
        CD_rht = self.CD0_ht_+CL_rht**2/K2
        CD_vt = self.CD0_vt_+CL_vt**2/K3
    
        # calc local velocities
        Vsq_lw = u_w_lw**2+v_w**2+w_w_lw**2
        Vsq_rw = u_w_rw**2+v_w**2+w_w_rw**2
        Vsq_lht = u_ht_lht**2+v_ht**2+w_ht_lht**2
        Vsq_rht = u_ht_rht**2+v_ht**2+w_ht_rht**2
        Vsq_vt = u_vt**2+v_vt**2+w_vt**2
    
        # constants, elemental areas for wings, ht, vt
        K = 0.5*self.rho_
        A_w = self.w_el_*self.chord_w_
        A_ht = self.ht_el_*self.chord_ht_
        A_vt = self.vt_el_*self.chord_vt_
    
        # calc lift force in wings, ht, vt
        LIFT_LW = CL_lw*K*Vsq_lw*np.flip(A_w, 0)
        LIFT_RW = CL_rw*K*Vsq_rw*A_w
        LIFT_LHT = CL_lht*K*Vsq_lht*np.flip(A_ht, 0)
        LIFT_RHT = CL_rht*K*Vsq_rht*A_ht
        LIFT_VT = CL_vt*K*Vsq_vt*A_vt
    
        # calc drag force in wings, ht, vt
        DRAG_LW = CD_lw*K*Vsq_lw*np.flip(A_w, 0)
        DRAG_RW = CD_rw*K*Vsq_rw*A_w
        DRAG_LHT = CD_lht*K*Vsq_lht*np.flip(A_ht, 0)
        DRAG_RHT = CD_rht*K*Vsq_rht*A_ht
        DRAG_VT = CD_vt*K*Vsq_vt*A_vt
    
        # calc pitching moments in wings, ht, vt
        #PITCH_LW = CM_lw*K*Vsq_lw*np.flip(A_ht, 0)*np.flip(self.chord_w_, 0)
        #PITCH_RW = CM_rw*K*Vsq_rw*A_w*self.chord_w_
        #PITCH_LHT = CM_lht*K*Vsq_lht*np.flip(A_ht, 0)*np.flip(self.chord_ht_, 0)
        #PITCH_RHT = CM_rht*K*Vsq_rht*A_ht*self.chord_ht_
        #PITCH_VT = CM_vt*K*Vsq_vt*A_vt*self.chord_vt_
    
        # total pitching moment due to lift and sweep'
        #TOTAL_PITCH = PITCH_LW+PITCH_RW+PITCH_LHT+PITCH_RHT+PITCH_VT
    
        # calc force in body X direction in wings, ht, vt
        LW_X = LIFT_LW*np.sin(alpha_lw)-DRAG_LW*np.cos(alpha_lw)
        RW_X = LIFT_RW*np.sin(alpha_rw)-DRAG_RW*np.cos(alpha_rw)
        LHT_X = LIFT_LHT*np.sin(alpha_lht)-DRAG_LHT*np.cos(alpha_lht)
        RHT_X = LIFT_RHT*np.sin(alpha_rht)-DRAG_RHT*np.cos(alpha_rht)
        VT_X = LIFT_VT*np.sin(alpha_vt)-DRAG_VT*np.cos(alpha_vt)
    
        # calc force in body Y direction in wings, ht, vt
        VT_Y = LIFT_VT*np.cos(alpha_vt)+DRAG_VT*np.sin(alpha_vt)
    
        # calc force in body Z direction in wings, ht, vt
        LW_Z = LIFT_LW*np.cos(alpha_lw)+DRAG_LW*np.sin(alpha_lw)
        RW_Z = LIFT_RW*np.cos(alpha_rw)+DRAG_RW*np.sin(alpha_rw)
        LHT_Z = LIFT_LHT*np.cos(alpha_lht)+DRAG_LHT*np.sin(alpha_lht)
        RHT_Z = LIFT_RHT*np.cos(alpha_rht)+DRAG_RHT*np.sin(alpha_rht)

        # Total body forces
        XF = float(thrust)+np.sum(LW_X)+np.sum(RW_X)+np.sum(LHT_X)+np.sum(RHT_X)+np.sum(VT_X)
        YF = np.sum(VT_Y)
        ZF = np.sum(LW_Z)+np.sum(RW_Z)+np.sum(LHT_Z)+np.sum(RHT_Z)

        # Moments about body X, Y, Z axes
        LM = np.sum(-lw*LW_Z-rw*RW_Z)+np.sum(-lht*LHT_Z-rht*RHT_Z)+np.sum(self.vtail_*VT_Y)
        MM = np.sum((LW_Z+RW_Z)*(self.x_cg_-self.x_ac_w_))+np.sum((LHT_Z+RHT_Z)*(self.x_cg_-self.x_ac_ht_))+\
                np.sum(self.vtail_*VT_X)#+np.sum(TOTAL_PITCH)
        NM = np.sum(-rw*RW_X-lw*LW_X)+np.sum(-rht*RHT_X-lht*LHT_X)
    
        print(XF, YF, ZF, LM, MM, NM)
        return [XF, YF, ZF, LM, MM, NM]

    # uses an interpolation function to calculate the exhaust velocity and thrust of the prop using momentum theory
    def thrustcalc( self,
                    power, 
                    u):
        if power>0:
            u2 = self.U2func(power, u)
            force = 0.5*rho*diskA*(u2**2-u**2)
        else:
            force = 0
        return force

    # calculates the chord of the wing at each point in its station
    def chord(  self,
                wing, 
                span, 
                area, 
                rc, 
                tc):
        k = tc/rc
        A = 2*area/((1+k)*span)
        B = 1*(1-k)/span
        res = A*(1-B*wing)
        return res

    # calculates the lift coefficient at each station along the wing
    def CL( self,
            wing, 
            dCL_da, 
            alpha, 
            CL0, 
            displacement, 
            dCL_de, 
            pos1, 
            pos2):
        aileronCL = self.heaviside(wing, pos1, pos2)
        stalled = (alpha >= self.alphamin_) & (alpha <= self.alphamax_)
        res = stalled.astype(int)*(CL0+dCL_da*alpha+aileronCL*dCL_de*displacement)
        return res

    # calculates the moment coefficient at each station along the wing
    def CM( self,
            wing, 
            dCM_da, 
            alpha, 
            CM0, 
            displacement, 
            dCM_de, 
            pos1, 
            pos2):
        aileronCL = self.heaviside(wing, pos1, pos2)
        stalled = (alpha >= alphamin) & (alpha <= alphamax)
        res = stalled.astype(int)*(CM0+dCM_da*alpha+aileronCL*dCM_de*displacement)
        return res

    # heaviside operator, returns a vector of 1s and 0s to make array operations easier
    def heaviside(  self,
                    wing, 
                    pos1, 
                    pos2):
        res = (wing >= pos1) & (wing <= pos2)
        return res.astype(int)

    # leap frog integrator to calculate accelerations velocities in the body frame, and calc displacement'
    # in the inertial frame
    def nlti(   self, 
                u, 
                v, 
                w, 
                p, 
                q, 
                r, 
                x, 
                y, 
                z, 
                phi, 
                theta, 
                psi, 
                A):
        # linear accelerations in the body frame
        du_dt = float(A[0]/self.m_-self.g_*np.sin(theta)-q*w+r*v)
        dv_dt = float(A[1]/self.m_+self.g_*np.cos(theta)*np.sin(phi)-r*u+p*w)
        dw_dt = float(A[2]/self.m_+self.g_*np.cos(theta)*np.cos(phi)-p*v+q*u)
    
        # angular accelerations in the body frame
        dp_dt = float(A[3]/self.Ixx_-(self.Izz_-self.Iyy_)/self.Ixx_*q*r)
        dq_dt = float(A[4]/self.Iyy_-(self.Ixx_ - self.Izz_)/self.Iyy_*r*p)
        dr_dt = float(A[5]/self.Izz_-(self.Iyy_ - self.Ixx_)/self.Izz_*p*q)
    
        # half time step representation of linear velocities
        u += 0.5*(self.udot_+du_dt)*self.timestep_
        v += 0.5*(self.vdot_+dv_dt)*self.timestep_
        w += 0.5*(self.wdot_+dw_dt)*self.timestep_
    
        # half time step representation of angular velocities
        p += 0.5*(self.pdot_+dp_dt)*self.timestep_
        q += 0.5*(self.qdot_+dq_dt)*self.timestep_
        r += 0.5*(self.rdot_+dr_dt)*self.timestep_
    
        # using cosine matrices to convert velocities and accelerations to inertial frame
        # (is there a better way to handle accelerations?)
        I = self.lindcm([-phi, -theta, -psi], [du_dt, dv_dt, dw_dt])
        X = self.lindcm([-phi, -theta, -psi], [u, v, w])
        J = self.angdcm([-phi, -theta, -psi], [dp_dt, dq_dt, dr_dt])
        W = self.angdcm([-phi, -theta, -psi], [p, q, r])
    
        # linear displacements in the inertial frame
        x += X[0]*self.timestep_+0.5*I[0]*self.timestep_**2
        y += X[1]*self.timestep_+0.5*I[1]*self.timestep_**2
        z += X[2]*self.timestep_+0.5*I[2]*self.timestep_**2
    
        # angular displacements in the inertial frame
        phi += W[0]*self.timestep_+0.5*J[0]*self.timestep_**2
        theta += W[1]*self.timestep_+0.5*J[1]*self.timestep_**2
        psi += W[2]*self.timestep_+0.5*J[2]*self.timestep_**2
    
        # store velocities so that in the next step, the half time step velocities can be calculated'
        self.udot_ = du_dt
        self.vdot_ = dv_dt
        self.wdot_ = dw_dt
        self.pdot_ = dp_dt
        self.qdot_ = dq_dt
        self.rdot_ = dr_dt

        return [u, v, w, 
                p, q, r, 
                x, y, z, 
                phi, theta, psi]

    # direction cosine matrix function
    def lindcm( self,
                A, 
                B):
        phi = A[0]
        theta = A[1]
        psi = A[2]

        DCM = np.array([[np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
                        [np.sin(phi)*np.sin(theta)*np.cos(psi)-np.cos(phi)*np.sin(psi), 
                        np.sin(phi)*np.sin(theta)*np.sin(psi)+np.cos(phi)*np.cos(psi), 
                        np.sin(phi)*np.cos(theta)],
                        [np.cos(phi)*np.sin(theta)*np.cos(psi)+np.sin(phi)*np.sin(psi),
                        np.cos(phi)*np.sin(theta)*np.sin(psi)-np.sin(phi)*np.cos(psi), 
                        np.cos(phi)*np.cos(theta)]])
        transform = np.dot(np.transpose(DCM), np.array(B))
        return transform

    # angular cosine matrix function
    def angdcm( self,
                A, 
                B):
        phi = A[0]
        theta = A[1]

        ACM = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        W = np.dot(ACM, np.array(B))
        return W

    # calculate body force and moment coefficients
    def coefs(  self,
                u, 
                v, 
                w, 
                A):
        XF, YF, ZF = A[0], A[1], A[2]
        LM, MM, NM = A[3], A[4], A[5]
        q = 0.5*self.rho_*(u**2+v**2+w**2)
        CX = XF/q/self.Sref_w_
        CY = YF/q/self.Sref_w_
        CZ = ZF/q/self.Sref_w_
        CL = LM/q/self.Sref_w_/self.wspan_
        CM = MM/q/self.Sref_w_/self.cbar_w_
        CN = NM/q/self.Sref_w_/self.wspan_
        return [CX, CY, CZ, CL, CM, CN]

    def plotaircraft(   ax,
                        X):
        x, y, z = X[0:3]

        # generate geometry using sizing
        # xltedge = 
        yltedge = np.linspace(-self.wspan/2, 0, 50)
        zltedge = np.zeros_like(yltedge)
        # xrtedge = 
        yrtedge = np.linspace(0, self.wspan/2, 50)
        zrtedge = np.zeros_like(yrtedge)
        xltip = np.linspace(0, tc_w,50)
        # yltip
        zltip = np.zeros_like(xrtip)
        xrtip = np.linspace(0, tc_w,50)
        # yrtip
        # xlledge =
        # xrledge =
        # ylledge =
        # yrledge =
    
        # rotate geometry

        # plot geometry
