import numpy as np

def fSetTiming():
    
    class myTime(object):
        T=None
        TotalTime=None
        signal_pulse_width=None
        T1=None
        T2=None
        T3=None
        T4=None
        T5=None
        T6=None
        h=None
        N_Loop=None

    
    out = myTime()

    out.T = 20e-3 # sec
    out.TotalTime = 200 # sec
    out.signal_pulse_width   = 4 * out.T #% 0.2; % sec 0.20

    # T1 = 3.0e0; #sec
    # T2 = T1 + signal_pulse_width ; #sec

    w2=5
    out.T1 = 1.0e0 #sec
    out.T2 = out.T1 + out.signal_pulse_width  #sec

    out.T3 = 1.0e0 + 0.15#sec
    out.T4 = out.T3 + w2*out.signal_pulse_width  #sec

#    out.T5 = 10.0e0 #sec
 #   out.T6 = out.T5 + w2*out.signal_pulse_width  #sec


    out.h = out.T/20.0

    out.N_Loop = np.fix(out.TotalTime / out.T) # 1000; # 20;
    
    return out;



def fPrintTiming(myT):
    """
    T=None
    TotalTime=None
    signal_pulse_width=None
    T1=None
    T2=None
    T3=None
    T4=None
    T5=None
    T6=None
    h=None
    N_Loop=None
    """
    print 'my time data are .......'\
    '\n myT.T=', myT.T,\
    '\n myT.TotalTime=',myT.TotalTime, \
    '\n myT.signal_pulse_width=',myT.signal_pulse_width,\
    '\n myT.T1=',myT.T1, \
    '\n myT.T2=',myT.T2, \
    '\n myT.T3=',myT.T3, \
    '\n myT.T4=',myT.T4, \
    '\n myT.T5=',myT.T5, \
    '\n myT.T6=',myT.T6, \
    '\n myT.h=',myT.h, \
    '\n myT.N_Loop=',myT.N_Loop,\
    '\n...................'
    
    

