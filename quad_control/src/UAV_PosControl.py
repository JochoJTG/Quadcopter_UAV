import rospy
import numpy
import math
from geometry_msgs.msg import Vector3

def callbackpos(data):
    global x
    global y
    global z
    x=data.x
    y=data.y
    z=data.z

def callbackang(data):
    global pitch
    global roll
    global yaw
    pitch=data.x
    roll=data.y
    yaw=data.z

def callbackvel(data):
    global xvel
    global yvel
    global zvel
    xvel=data.x
    yvel=data.y
    zvel=data.z

def callbackposdes(data):
    global xdes
    global ydes
    global zdes
    xdes=data.x
    ydes=data.y
    zdes=data.z

def callbackveldes(data):
    global xveldes
    global yveldes
    global zveldes
    xveldes=data.x
    yveldes=data.y
    zveldes=data.z
 

def main():

    # Inicializar ROS y la creación del nodo
    rospy.init_node('PositionControl', anonymous=True)
    
    # Declaración de los publishers
    rospy.Subscriber("dynamics/pos",Vector3, callbackpos)
    rospy.Subscriber("dynamics/attitude",Vector3, callbackang)
    rospy.Subscriber("dynamics/lin_vel",Vector3, callbackvel)
    rospy.Subscriber("refdata/posdes",Vector3, callbackposdes)
    rospy.Subscriber("refdata/veldes",Vector3, callbackveldes)
    angdes = rospy.Publisher('posdata/angdes',Vector3, queue_size=10)
    Th = rospy.Publisher('posdata/thrust',Vector3, queue_size = 10)

    rate = rospy.Rate(100)  
    kpx=1
    kdx=1
    kpy=1
    kdy=1
    kpz=1
    kdz=1
    m=2
    angs=Vector3()




    while not rospy.is_shutdown():

        ex=x-xdes
        ey=y-ydes
        ez=z-zdes
        exvel=xvel-xveldes
        eyvel=yvel-yveldes
        ezvel=zvel-zveldes
    
        uvx=-kpx*ex-kdx*exvel
        uvy=-kpy*ey-kdy*eyvel
        uvz=-kpz*ez-kdz*ezvel

        thrust=(m/(math.cos(pitch)*math.cos(roll))*(-9.81+uvz))
        phides=math.asin((m/thrust)*(math.sin(yaw)*uvx-math.cos(yaw)*uvy))
        thetades=math.asin((m/thrust)*uvx-math.sin(yaw)*math.sin(phides)/(math.cos(yaw)*math.cos(phides)))
        
        angs.x= thetades
        angs.y= phides
        angs.z=thrust


        

        angdes.publish(angs)
  



        #xt=xdes-x
        #yt=ydes-y
        #pitcherror=xt*math.cos(yaw)-yt*math.sin(yaw)
        #rollerror=xt*math.sin(yaw)-yt*math.cos(yaw)

        #poserror=math.sqrt(pow(x-xdes,2)+pow(y-ydes,2))
        #velerror=math.sqrt(pow(xvel-xveldes,2)+pow(yvel-yveldes,2))
        #angerror=math.atan2(x-xdes,y-ydes)
        #heighterror=z-zdes

        #thrust=heighterror*2
        #rolldes=rollerror
        #pitchdes=pitcherror

        rate.sleep()

#

if __name__ == '__main__':
    try:
        x=0
        y=0
        z=0
        pitch=0
        roll=0
        yaw=0
        xvel=0
        yvel=0
        zvel=0
        xdes=0
        ydes=0
        zdes=100
        xveldes=0
        yveldes=0
        zveldes=0
        main()
    except rospy.ROSInterruptException:
        pass