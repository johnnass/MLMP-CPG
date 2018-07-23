from random import randint
#from enum import Enum
import math


##########################
class Joint(object):
    name =                      'JointName'
    maxJointAng =           0.0 # [Rad]
    minJointAng =            0.0 # [Rad] 
    init_motor_pos =        0.0 # [Rad] 
    gain_joint_motor =     5.0
    joint_motor_signal =   init_motor_pos # [Rad]

class RSNeuron(object): # Roat Selverston Neuron 
    Es  = 0.0
    A_f = 5.0
    InjCurrent_value = 0.0
    InjCurrent_MultiplicationFactor = 1.0

    sigma_f = 0.0 
    sigma_s = 0.0

    tau_m = 0.1 # * ((T*1000)/10); # Change it will change the joint velocity. 
    tau_s = 20 * tau_m

    V_0 = 0
    q_0 = 0
        
    V = V_0 
    q = q_0 
    out = V
        
class RGLayer(object):
    def __init__(self):
        self.E = RSNeuron()
        self.F = RSNeuron() 

class Sigmoid(object): 
    alpha  = 0.0
    theta = 0.0

class PFNeuron(object):
    alpha = 0.5
    theta = 0.0

    alpha_MLR = 1
    theta_MLR = 0

    i = 0.0
    o = 0.0

class PFLayer(object):
        def __init__(self):
            self.E = PFNeuron()
            self.F = PFNeuron()       
    
class SNeuron(object): 
    alpha = 5
    theta = 0.0

    o = 0.0

class SNLayer(object):
    def __init__(self):
        self.E = SNeuron()
        self.F = SNeuron()

class MNeuron(object): 
    alpha = 5
    theta = 0.5

    i = 0.0
    o = 0.0
        
class MNLayer(object):
    def __init__(self):
        self.E = MNeuron()
        self.F = MNeuron()

class mlmpCPG(object):
    #CPG Structure
    #
    description ='cpg description'
    # RG 2 PF
    W_F_RG2PF = 1
    W_E_RG2PF = 1
    # PF 2 MN
    W_F_PF2MN = 1
    W_E_PF2MN = 1
    # SN 2 MN
    W_F_SN2MN = -0
    W_E_SN2MN = -0


    def __init__(self):
        # Joint 
        self.joint = Joint()
        # RG
        self.RG = RGLayer()
        # PF 
        self.PF = PFLayer()
        # MN
        self.MN = MNLayer()
        # SN
        self.SN = SNLayer()
    
    #########################

    def fSetPatternPF(self,pfPat):
    
        self.PF.E.alpha = pfPat.alpha
        self.PF.E.theta = pfPat.theta

        self.PF.F.alpha = pfPat.alpha
        self.PF.F.theta = pfPat.theta

    #########################

    def fSetPatternRG(self,rgPat):
    
        self.RG.E.sigma_f = rgPat.sigma_f 
        self.RG.E.sigma_s = rgPat.sigma_s 
        self.RG.E.InjCurrent_MultiplicationFactor = rgPat.InjCurrentMultiplicationFactor
        self.RG.E.tau_m = rgPat.TAU_M
        self.RG.E.tau_s = 20 * self.RG.E.tau_m

        self.RG.F.sigma_f = rgPat.sigma_f 
        self.RG.F.sigma_s = rgPat.sigma_s 
        self.RG.F.InjCurrent_MultiplicationFactor = rgPat.InjCurrentMultiplicationFactor
        self.RG.F.tau_m = rgPat.TAU_M
        self.RG.F.tau_s = 20 * self.RG.E.tau_m
        return; 
        
    #########################

    def fSetSensorNeurons(self,filename2):
        
        #self.SN.E.theta = (self.joint.maxJointAng+self.joint.minJointAng)/2.0
        #self.SN.F.theta = self.SN.E.theta
        
        self.SN.E.alpha = 5
        self.SN.F.alpha = 5 
        
        ctr=0
        with open(filename2) as input:
            for line in input:
                jointName, positivSide = (item.strip() for item in line.split(' ',1))
                if jointName==self.description :
                    if positivSide=='E':
                        self.SN.F.alpha = -1*math.fabs(self.SN.F.alpha)
                        self.SN.E.theta = self.joint.maxJointAng
                        self.SN.F.theta = self.joint.minJointAng

                    if positivSide=='F':
                        self.SN.E.alpha = -1*math.fabs(self.SN.E.alpha)
                        self.SN.E.theta = self.joint.minJointAng
                        self.SN.F.theta = self.joint.maxJointAng

            ctr +=1 

        
        #MyNaoPsitiveAngle_E_or_F
        
        #for i in range(0, len(Net)):
        #print 'Net[',self.description,'].SN.E.theta :', self.SN.E.theta
        #print 'Net[',self.description,'].SN.F.theta :', self.SN.F.theta

        #print 'Net[',self.description,'].SN.E.alpha :', self.SN.E.alpha
        #print 'Net[',self.description,'].SN.F.alpha :', self.SN.F.alpha
        #print '\n'

        return; 
    
    #########################
    def fUpdateInitPos(self,CurPos):
        self.joint.init_motor_pos = CurPos
        #print CurPos
        #print("%.3f"% (CurPos))
        return; 
    
    #########################
    def fUpdateLocomotionNetwork(self,myT,JointPos):
        
        self.fUpdateLocomotionNetworkSN(JointPos)
        self.fUpdateLocomotionNetworkRG(myT)
        self.fUpdateLocomotionNetworkPF()
        self.fUpdateLocomotionNetworkMN()
        self.fCalcMotorCommand()
        return; 
    #########################
    def fUpdateLocomotionNetworkSN(self,JointPos):
        
        Ealpha = self.SN.E.alpha
        Falpha = self.SN.F.alpha
        Etheta = self.SN.E.theta
        Ftheta = self.SN.F.theta
        
        self.SN.E.o = 1.0/( 1.0+math.exp( Ealpha*(Etheta - JointPos) ) )
        self.SN.F.o = 1.0/( 1.0+math.exp( Falpha*(Ftheta - JointPos) ) ) 
        
        return; 
    #########################
    #########################
    def fUpdateLocomotionNetworkRG(self,myT):
        
        # E
        tau_m_E = self.RG.E.tau_m ;
        tau_s_E = self.RG.E.tau_s ;

        sigma_f_E = self.RG.E.sigma_f; 
        sigma_s_E = self.RG.E.sigma_s; 

        A_f_E = self.RG.E.A_f ;
        Es_E  = self.RG.E.Es ;
        V_E = self.RG.E.V ;
        q_E = self.RG.E.q ;
        InjCurr_E = self.RG.E.InjCurrent_value ;


        # F
        tau_m_F = self.RG.F.tau_m ;
        tau_s_F = self.RG.F.tau_s ;

        sigma_f_F = self.RG.F.sigma_f; 
        sigma_s_F = self.RG.F.sigma_s; 

        A_f_F = self.RG.F.A_f ;
        Es_F  = self.RG.F.Es ;
        V_F = self.RG.F.V ;
        q_F = self.RG.F.q ;
        InjCurr_F = self.RG.F.InjCurrent_value ;
        
        for J in range(0,10):
            # (-1/tm)*( V-Af*tanh(sigma_s/A_f*V)+q-InjCurr);
            # dq/dt = (1/tau_s)*(-q_E+sigma_s*(V-Es));

            k1 = myT.h * ( (-1/tau_m_F) * ( (V_F+0   ) - A_f_F*math.tanh( (sigma_f_F/A_f_F)*(V_F+0   ) ) + (q_F+0   ) - InjCurr_F ) )
            l1 = myT.h * ( ( 1/tau_s_F) * (-(q_F+0   ) + sigma_s_F * ( V_F - Es_F ) ) )

            k2 = myT.h * ( (-1/tau_m_F) * ( (V_F+k1/2) - A_f_F*math.tanh( (sigma_f_F/A_f_F)*(V_F+k1/2) ) + (q_F+l1/2) - InjCurr_F ) )
            l2 = myT.h * ( ( 1/tau_s_F) * (-(q_F+l1/2) + sigma_s_F * ( V_F - Es_F ) ) )
        
            k3 = myT.h * ( (-1/tau_m_F) * ( (V_F+k2/2) - A_f_F*math.tanh( (sigma_f_F/A_f_F)*(V_F+k2/2) ) + (q_F+l2/2) - InjCurr_F ) )
            l3 = myT.h * ( ( 1/tau_s_F) * (-(q_F+l2/2) + sigma_s_F * ( V_F - Es_F ) ) )
        
            k4 = myT.h * ( (-1/tau_m_F) * ( (V_F+k3  ) - A_f_F*math.tanh( (sigma_f_F/A_f_F)*(V_F+k3  ) ) + (q_F+l3  ) - InjCurr_F ) )
            l4 = myT.h * ( ( 1/tau_s_F) * (-(q_F+l3  ) + sigma_s_F * ( V_F - Es_F ) ) )

            k = 1/6.0 * ( k1 + 2*k2 + 2*k3 + k4 )
            l = 1/6.0 * ( l1 + 2*l2 + 2*l3 + l4 )

            V_F = V_F+k
            q_F = q_F+l

            ######

            # (-1/tm)*( V-Af*tanh(sigma_s/A_f*V)+q-InjCurr);
            # dq_E/dt = (1/tau_s)*(-q+sigma_s*(V-Es));

            kk1 = myT.h * ( (-1/tau_m_E) * ( (V_E+0   ) - A_f_E*math.tanh( (sigma_f_E/A_f_E)*(V_E+0   ) ) + (q_E+0   ) - InjCurr_E ) )
            ll1 = myT.h * ( ( 1/tau_s_E) * (-(q_E+0   ) + sigma_s_E * ( V_E - Es_E ) ) )

            kk2 = myT.h * ( (-1/tau_m_E) * ( (V_E+kk1/2) - A_f_E*math.tanh( (sigma_f_E/A_f_E)*(V_E+kk1/2) ) + (q_E+ll1/2) - InjCurr_E ) )
            ll2 = myT.h * ( ( 1/tau_s_E) * (-(q_E+l1/2) + sigma_s_E * ( V_E - Es_E ) ) )
        
            kk3 = myT.h * ( (-1/tau_m_E) * ( (V_E+kk2/2) - A_f_E*math.tanh( (sigma_f_E/A_f_E)*(V_E+kk2/2) ) + (q_E+ll2/2) - InjCurr_E ) )
            ll3 = myT.h * ( ( 1/tau_s_E) * (-(q_E+ll2/2) + sigma_s_E * ( V_E - Es_E ) ) )
        
            kk4 = myT.h * ( (-1/tau_m_E) * ( (V_E+kk3  ) - A_f_E*math.tanh( (sigma_f_E/A_f_E)*(V_E+kk3  ) ) + (q_E+ll3  ) - InjCurr_E ) )
            ll4 = myT.h * ( ( 1/tau_s_E) * (-(q_E+ll3  ) + sigma_s_E * ( V_E - Es_E ) ) )

            kk = 1/6.0 * ( kk1 + 2*kk2 + 2*kk3 + kk4 )
            ll = 1/6.0 * ( ll1 + 2*ll2 + 2*ll3 + ll4 )

            V_E = V_E+kk
            q_E = q_E+ll

        self.RG.F.V = V_F
        self.RG.F.q = q_F
        self.RG.F.out = V_F

        self.RG.E.V = V_E
        self.RG.E.q = q_E
        self.RG.E.out = V_E

        return; 
    #########################
    def fUpdateLocomotionNetworkPF(self):
        
        
        self.PF.E.i = self.RG.E.out * self.W_E_RG2PF 
        self.PF.F.i = self.RG.F.out * self.W_F_RG2PF 
        
        self.PF.E.o = 1/(1+math.exp(self.PF.E.alpha*self.PF.E.alpha_MLR*((self.PF.E.theta+self.PF.E.theta_MLR)-self.PF.E.i)))
        self.PF.F.o = 1/(1+math.exp(self.PF.F.alpha*self.PF.F.alpha_MLR*((self.PF.F.theta+self.PF.F.theta_MLR)-self.PF.F.i)))
        return; 
    #########################
    def fUpdateLocomotionNetworkMN(self):
        
        #self.MN.E.i = self.PF.E.o * self.W_E_PF2MN
        #self.MN.F.i = self.PF.F.o * self.W_F_PF2MN

        self.MN.E.i = (self.PF.E.o * self.W_E_PF2MN + self.SN.E.o * self.W_E_SN2MN)/2.0
        self.MN.F.i = (self.PF.F.o * self.W_F_PF2MN + self.SN.F.o * self.W_F_SN2MN)/2.0

        
        self.MN.E.o = 1/( 1+math.exp( self.MN.E.alpha*(self.MN.E.theta - self.MN.E.i) ) )
        self.MN.F.o = 1/( 1+math.exp( self.MN.F.alpha*(self.MN.F.theta - self.MN.F.i) ) ) 
        return; 
    #########################
    def fSetMotorPolarity(self,filename2):
        
        with open(filename2) as input:
            for line in input:
                jointName, positivSide = (item.strip() for item in line.split(' ',1))
                if jointName==self.description :
                    if positivSide=='F':
                        self.joint.gain_joint_motor = -1*math.fabs(self.joint.gain_joint_motor)
                        # please refer to the function "fCalcMotorCommand" to check the subtraction 
        return; 
    
    #########################    
    def fCalcMotorCommand(self):
        
        init_motor_pos = self.joint.init_motor_pos
        gain_joint_motor = self.joint.gain_joint_motor
        Eo = self.MN.E.o
        Fo = self.MN.F.o 
        self.joint.joint_motor_signal = init_motor_pos  + ( gain_joint_motor * (Eo - Fo) )
        
        #self.joint.joint_motor_signal = self.joint.init_motor_pos  + ( self.joint.gain_joint_motor * (self.MN.E.o - self.MN.F.o) )
        return; 
    #########################
    def fPrint(self):
        #print '\n'        
        print '---------------------'
        print '-1- Description :', self.description 
        print '\n'        
        print '-2- joint name :', self.joint.name
        print '-3- joint maxJointAng :', self.joint.maxJointAng
        print '-4- joint minJointAng :', self.joint.minJointAng
        print '-5- joint init_motor_pos :', self.joint.init_motor_pos
        print '-6- joint gain_joint_motor :', self.joint.gain_joint_motor
        print '-7- joint motor_signal :', self.joint.joint_motor_signal
        print '\n'        
        print '-8- RG E Es:', self.RG.E.Es
        print '-9- RG E A_f:', self.RG.E.A_f
        print '-10- RG E InjCurrent_value:', self.RG.E.InjCurrent_value
        print '-11- RG E InjCurrent_MultiplicationFactor:', self.RG.E.InjCurrent_MultiplicationFactor
        print '-12- RG E sigma_f:', self.RG.E.sigma_f
        print '-13- RG E sigma_s:', self.RG.E.sigma_s
        print '-14- RG E tau_m:', self.RG.E.tau_m
        print '-15- RG E tau_s:', self.RG.E.tau_s
        print '-16- RG E V_0:', self.RG.E.V_0
        print '-17- RG E q_0:', self.RG.E.q_0
        print '-18- RG E V:', self.RG.E.V
        print '-19- RG E q:', self.RG.E.q
        print '-20- RG E out:', self.RG.E.out
        print '\n'        
        print '-21- RG F Es:', self.RG.F.Es
        print '-22- RG F A_f:', self.RG.F.A_f
        print '-23- RG F InjCurrent_value:', self.RG.F.InjCurrent_value
        print '-24- RG F InjCurrent_MultiplicationFactor:', self.RG.F.InjCurrent_MultiplicationFactor
        print '-25- RG F sigma_f:', self.RG.F.sigma_f
        print '-26- RG F sigma_s:', self.RG.F.sigma_s
        print '-27- RG F tau_m:', self.RG.F.tau_m
        print '-28- RG F tau_s:', self.RG.F.tau_s
        print '-29- RG F V_0:', self.RG.F.V_0
        print '-30- RG F q_0:', self.RG.F.q_0
        print '-31- RG F V:', self.RG.F.V
        print '-32- RG F q:', self.RG.F.q
        print '-33- RG F out:', self.RG.F.out
        print '\n'
        print '-34- PF E alpha:', self.PF.E.alpha
        print '-35- PF E theta:', self.PF.E.theta
        print '-36- PF E alpha_MLR:', self.PF.E.alpha_MLR
        print '-37- PF E theta_MLR:', self.PF.E.theta_MLR
        print '-38- PF E i:', self.PF.E.i
        print '-39- PF E o:', self.PF.E.o
        print '\n'
        print '-40- PF F alpha:', self.PF.F.alpha
        print '-41- PF F theta:', self.PF.F.theta
        print '-42- PF F alpha_MLR:', self.PF.F.alpha_MLR
        print '-43- PF F theta_MLR:', self.PF.F.theta_MLR
        print '-44- PF F i:', self.PF.F.i
        print '-45- PF F o:', self.PF.F.o    
        print '\n'
        print '-46- MN E alpha :', self.MN.E.alpha    
        print '-47- MN E theta:', self.MN.E.theta
        print '-48- MN E i:', self.MN.E.i
        print '-49- MN E o:', self.MN.E.o
        print '\n'
        print '-50- MN F alpha :', self.MN.F.alpha    
        print '-51- MN F theta:', self.MN.F.theta
        print '-52- MN F i:', self.MN.F.i
        print '-53- MN F o:', self.MN.F.o
        print '\n'        
        print '-54- SN F alpha :', self.SN.F.alpha    
        print '-55- SN F theta:', self.SN.F.theta
        print '-56- SN F o:', self.SN.F.o
        print '\n'        
        print '-57- SN E alpha :', self.SN.E.alpha    
        print '-58- SN E theta:', self.SN.E.theta
        print '-59- SN E o:', self.SN.E.o
        print '\n'
        print '-60- W_F_RG2PF:', self.W_F_RG2PF  
        print '-61- W_E_RG2PF:', self.W_E_RG2PF
        print '-62- W_F_PF2MN:', self.W_F_PF2MN
        print '-63- W_E_PF2MN:', self.W_E_PF2MN
        print '-64- W_F_SN2MN:', self.W_F_SN2MN
        print '-65- W_E_SN2MN:', self.W_E_SN2MN

        return; 



#########################
#########################
class RG_Patterns(object): 
        def __init__(self,Sf,Ss,InjCMF,TM):
            self.sigma_f = Sf
            self.sigma_s = Ss
            self.InjCurrentMultiplicationFactor = InjCMF
            self.TAU_M = TM

        def fPrint(self):
            print '---------------------'            
            print 'Pattern_sigma_f=',self.sigma_f,'\nPattern_sigma_s=',self.sigma_s,\
            '\nPattern_InjCurrentMultiplicationFactor=',self.InjCurrentMultiplicationFactor,\
            '\nPattern_TAU_M=',self.TAU_M
            return; 
            

#########################
class PF_Patterns(object): 

        def __init__(self,Aa,Ta):
            self.alpha = Aa
            self.theta = Ta

        def fPrint(self):
            print '---------------------'
            print 'Pattern_alpha=',self.alpha,'\nPattern_theta=',self.theta
            return; 
            
#########################
        
def fnewMLMPcpg(nbr_cpg):
    #cpgJ = [ mlmpCPG() for i in range(nbr_cpg)]
    cpgJ = []
    for i in range(nbr_cpg) :
        cpgJ.append(mlmpCPG())
    
    return cpgJ; 

###################################

def fSetCPGNet(CPG,filename,filename2):
    # EXAMPLE
    # 1 HeadYaw              -2.0857  2.0857
    # 2 HeadPitch            -0.6720  0.5149
    
    Net = CPG
    info = {} 
    ctr=0
    with open(filename) as input:
        for line in input:
            jointName, mymin, mymax = (item.strip() for item in line.split(' ',2))
            #print jointName,': MIN ', mymin,'  MAX ', mymax
            Net[ctr].joint.minJointAng = float(mymin)
            Net[ctr].joint.maxJointAng = float(mymax)
            Net[ctr].joint.name = jointName
            Net[ctr].description = jointName

            ctr +=1 
    
    for i in range(0, len(Net)):
        Net[i].fSetSensorNeurons(filename2)
        
    for i in range(0, len(Net)):
        Net[i].fSetMotorPolarity(filename2)
        
    return Net;

##################################

