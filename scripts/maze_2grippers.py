import time
import openravepy
import rospkg
import rospy

#### YOUR IMPORTS GO HERE ####

def DrawPath(path, robot, color):
    for i in path:
        robot.SetActiveDOFValues(i)
        drawHandle.append(env.plot3(points=robot.GetLinks()[6].GetTransform()[0:3,3],pointsize = 0.05, colors =color,drawstyle=1))

drawHandle=[]

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'

    with env:
#        env.Load('scenes/gripper.env.xml')
        env.Load('scenes/maze2.env.xml')

    with env:
        r = env.ReadRobotXMLFile('scenes/gripper1.kinbody.xml')
        env.AddRobot(r)
        robot1 = env.GetRobot("Gripper1")
        robot1.SetActiveDOFs([0,1,2,3,4,5])
        robot1.SetActiveManipulator("dummy")
        robot1.SetActiveDOFValues([8.8,6.0,0.7,0.0,0.0,0.0])
    waitrobot(robot1)

    point = robot1.GetLinks()[6].GetTransform()[0:3,3]
#    print point

    with env:
        r = env.ReadRobotXMLFile('scenes/gripper2.kinbody.xml')
        env.AddRobot(r)
        robot2 = env.GetRobot("Gripper2")
        robot2.SetActiveDOFs([0,1,2,3,4,5])
        robot2.SetActiveManipulator("dummy")
        robot2.SetActiveDOFValues([8.4,6.8,0.7,0.0,0.0,0.0])
    waitrobot(robot2)


    ### INITIALIZE YOUR PLUGIN HERE ###

    RaveInitialize()
    RaveLoadPlugin('build/myproject')
    
    MyRRT = RaveCreateModule(env,'MyNewModule')
#    print MyRRTmodule.SendCommand('help')
#    print MyRRT.SendCommand('MyCommand')

    ### END INITIALIZING YOUR PLUGIN ###

    # tuck in the PR2's arms for driving
#    tuckarms(env,robot);

    with env:

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig

        print MyRRT.SendCommand('goal -0.2 -1.3 0.7 0.0 0.0 0.0')
        print MyRRT.SendCommand('goal -0.1 -1.5 0.7 0.0 0.0 0.0')
        print MyRRT.SendCommand('bias 0.1')
        print MyRRT.SendCommand('step 0.35')
        print MyRRT.SendCommand('SetUpperBound 10.0 10.0 0.9 pi pi pi')
        print MyRRT.SendCommand('SetLowerBound -3.0 -10.0 0.3 -pi -pi -pi')
        print MyRRT.SendCommand('isBiRRT 0')
        print MyRRT.SendCommand('SetWeight')
#        print MyRRT.SendCommand('SetBound')

#        print MyRRT.SendCommand('MyCommand')
        startTime = time.time()

#        print MyRRT.SendCommand('RRT')
        print MyRRT.SendCommand('BiRRT')

        rrtTime = time.time()-startTime
        print MyRRT.SendCommand('Smooth 500')
    waitrobot(robot2)


    time.sleep(5)

    with env:
        pathStr1 = MyRRT.SendCommand('GetRRTPath 0')

        path1 = []
        A = []
        B = []
        pathStr1 = pathStr1.split('\n')
        for i in xrange(len(pathStr1)-1):
            path1.append([])
            A.append(pathStr1[i].split(','))
            for j in xrange(len(A[i])):
                path1[i].append(float(A[i][j]))

        pathStr2 = MyRRT.SendCommand('GetRRTPath 1')

        path2 = []
        pathStr2 = pathStr2.split('\n')
        for i in xrange(len(pathStr2)-1):
            path2.append([])
            B.append(pathStr2[i].split(','))
            for j in xrange(len(B[i])):
                path2[i].append(float(B[i][j]))

        red = [1,0,0]
        blue = [0, 0, 1]
        DrawPath(path1, robot1, red)
        DrawPath(path2, robot2, blue)



        '''
#        startconfig = robot.GetActiveDOFValues()
        bias = 0.2
        stepsize = 0.2
        isBiRRT = 0;

        startTime = time.time()

        print MyRRT.SendCommand('goal 0.449 -0.201 -0.151 0.0 0.0 -0.11 0.0')
        print MyRRT.SendCommand('bias 0.06')
        print MyRRT.SendCommand('step 0.4')
        print MyRRT.SendCommand('isBiRRT 0')
        print MyRRT.SendCommand('SetWeight')
        print MyRRT.SendCommand('SetBound')

        pathStr=MyRRT.SendCommand('RRT')
#        pathStr=MyRRT.SendCommand('BiRRT')
#        print robot.GetActiveDOFLimits();
#        print robot.GetActiveDOFWeights()
        '''

        '''
        lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
        if not lmodel.load():
            lmodel.autogenerate()

        lmodel.setRobotWeights()
        lmodel.setRobotResolutions(xyzdelta=0.01)
        print 'robot resolutions: ', repr(robot.GetDOFResolutions())
        print 'robot weights: ', repr(robot.GetActiveDOFWeights())
        '''



        trajectory1 = RaveCreateTrajectory(env,'')
        config1 = robot1.GetActiveConfigurationSpecification('linear')
        trajectory1.Init(config1)

        for i in xrange(len(path1)):
                trajectory1.Insert(i,path1[i])

        planningutils.RetimeActiveDOFTrajectory(trajectory1,robot1,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
#        robot1.GetController().SetPath(trajectory1)

        trajectory2 = RaveCreateTrajectory(env,'')
        config2 = robot2.GetActiveConfigurationSpecification('linear')
        trajectory2.Init(config2)

        for i in xrange(len(path2)):
                trajectory2.Insert(i,path2[i])

        planningutils.RetimeActiveDOFTrajectory(trajectory2,robot2,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
#        robot2.GetController().SetPath(trajectory2)




        ### END OF YOUR CODE ###
    waitrobot(robot2)

    print "Planning time: "
    print rrtTime


    raw_input("Press enter to exit...")

