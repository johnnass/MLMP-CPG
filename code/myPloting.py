import matplotlib.pyplot as plt

def fPlotJointCommandSensor(All_Command,All_Sensor,JointIndex,Str):
    
    data2print1=[]
    for i in range(0,len(All_Command)):
        data2print1.append(All_Command[i][JointIndex])
    
    data2print2=[]
    for i in range(0,len(All_Sensor)):
        data2print2.append(All_Sensor[i][JointIndex])
    
    plt.plot( range(len(data2print1)), data2print1,'b', range(len(data2print2)),data2print2,'r')
    plt.xlabel('Time')
    plt.ylabel('Motor Command / Sensor Value')
    plt.title(Str)
    plt.grid(True)
    plt.show()
        
"""
    # row and column sharing
    f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    ax1.plot( range(len(date2print1)), date2print1,'b', range(len(date2print2)),date2print2,'r')
    #ax1.set_title('Sharing x per column, y per row')
    ax2.plot( range(len(date2print1)), date2print1,'b', range(len(date2print2)),date2print2,'r')
    ax3.plot( range(len(date2print1)), date2print1,'b', range(len(date2print2)),date2print2,'r')
    ax4.plot( range(len(date2print1)), date2print1,'b', range(len(date2print2)),date2print2,'r')
    plt.grid(True)
    plt.show()

"""
