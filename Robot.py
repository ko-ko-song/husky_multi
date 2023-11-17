import threading
import rospy
from value import ServiceResult, RobotID, ObjectID, Direction, RobotStatus
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from mapManager import MapManager
from communication.messageFactory import MessageFactory
from service.service import *
from communication.adaptor import Adaptor
import tf.transformations
from publisher import Publisher
import math 

import numpy as np
import math

from service.service import LoadService, UnloadService
from configuration import RobotConfiguration



class Position():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class Robot():
    def __init__(self, robotName, adaptor):
        self.robotName = robotName 
        self.status = RobotStatus.Ready
        self.loading = False
        self.velocity = 0.5
        
        self.theta = 0.0
        self.battery = 100.0
        self.thresholdMeter = 0.3
        self.thresholdDegree = 2.0
        self.adaptor = adaptor
        self.behavior = None
        self.stop_behavior = False
        self.speed = Twist()
        self.position = Position()
        self.liftPositionZ = 0.0
        
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStatesCallback, queue_size=10)
        self.speedPublisher = rospy.Publisher("/robot" + str(self.robotName.value) + "/husky_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.liftPositionPublisher = rospy.Publisher("/robot" + str(self.robotName.value) + "/lift_joint_position_controller/command", Float64, queue_size=1)

    def modelStatesCallback(self, msg):
        names = msg.name
        robot_model_index = -1
        poses = msg.pose

        for index, name in enumerate(names):
            if name == "robot" + str(self.robotName.value):
                robot_model_index = index
                break

        if index != -1:
            pose = poses[robot_model_index]
            self.position.x = pose.position.x
            self.position.y = pose.position.y
            euler = tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            self.theta = euler[2]
            

    
        

    def executeNewBehavior(self, serviceInstance):
        if serviceInstance is not None:
            if self.thread is not None and self.thread.is_alive():
                self.stop_thread = True
                self.thread.join()  # 이전 스레드가 완전히 종료될 때까지 기다림

            # 새 작업 시작
            self.stop_thread = False
            self.thread = threading.Thread(target=self.executeBehavior, args=(serviceInstance,))
            self.thread.start()

    def executeBehavior(self, serviceInstance):
        if serviceInstance is None:
            return

        ackMessage = MessageFactory.newAckMessage(serviceInstance)
        self.adaptor.broadcast(ackMessage)
        
        if isinstance(serviceInstance, MoveService):
            self.handleMoveThroughService(serviceInstance)

        elif isinstance(serviceInstance, CancelMoveService):
            self.handleCancelMoveService(serviceInstance)

        elif isinstance(serviceInstance, GuideMoveService):
            self.handleMoveGuideService(serviceInstance)

        elif isinstance(serviceInstance, PreciseMoveService):
            self.handlePreciseMoveService(serviceInstance)

        elif isinstance(serviceInstance, FlatPreciseMoveService):
            self.handleFlatPreciseMoveService(serviceInstance)

        elif isinstance(serviceInstance, StraightBackMoveService):
            self.handleStraightBackMoveService(serviceInstance)

        elif isinstance(serviceInstance, LoadService):
            self.handleLoadService(serviceInstance)

        elif isinstance(serviceInstance, UnloadService):
            self.handleUnloadService(serviceInstance)

        else:
            raise Exception("undefined service type")
        serviceInstance.result = ServiceResult.Success
        ackEndMessage = MessageFactory.newAckEndMessage(serviceInstance)
        self.adaptor.broadcast(ackEndMessage)

    def handleMoveThroughService(self, serviceInstance):
        self.status = RobotStatus.Move

        for node in serviceInstance.path:
            self.moveToNode(node, 0.4)
        
        self.status = RobotStatus.Ready



    def handleCancelMoveService(self, serviceInstance):
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.speedPublisher.publish(self.speed)
        self.status = RobotStatus.Ready


    def handleMoveGuideService(self, serviceInstance):
        self.status = RobotStatus.Move

        if serviceInstance.direction is Direction.FORWARD:
            self.moveToNode(serviceInstance.node, 0.2)
        
        elif self.service.direction is Direction.BACKWARD:
            self.moveBackToNode(serviceInstance.node, 0.2)

        else:
            raise Exception()
        
        self.status = RobotStatus.Ready



    def handlePreciseMoveService(self, serviceInstance):
        self.status = RobotStatus.Move

        self.moveToNode(serviceInstance.node, 0.2)
        self.status = RobotStatus.Ready


    def handleFlatPreciseMoveService(self, serviceInstance):
        self.status = RobotStatus.Move

        self.moveToNode(serviceInstance.node, 0.2)
        self.status = RobotStatus.Ready

    def handleStraightBackMoveService(self, serviceInstance):
        self.status = RobotStatus.Move

        self.moveBackToNode(serviceInstance.node, 0.2)
        self.status = RobotStatus.Ready



    def handleLoadService(self, serviceInstance):
        self.status = RobotStatus.Loading

        self.loadObject()

        self.loading = True
        self.status = RobotStatus.Ready
        

    def handleUnloadService(self, serviceInstance):
        self.status = RobotStatus.Unloading

        self.unLoadObject()

        
        self.loading = True
        self.status = RobotStatus.Ready




    def isRightPosition(self, goal):
        distance = math.sqrt((goal.x - self.position.x) ** 2 + (goal.y - self.position.y) ** 2)
        if distance <= RobotConfiguration.DIST_THRESHOLD:
            # print('goal: {}, pose: {} -> checkPos : True'.format(goal.x, self.position.x))
            return True
        else:
            # print('goal: {}, pose: {} -> checkPos : False'.format(goal.x, self.position.x))
            return False



    def liftToTargetPosition(self, z):
        while not abs(self.liftPositionZ-z) < RobotConfiguration.LIFT_THRESHOLD:
            if self.stop_behavior == True:
                return

            if self.status != RobotStatus.Paused:
                self.battery = self.battery - 0.001
                time.sleep(0.1)
                msg = Float64()
                msg.data = z
                self.liftPositionPublisher.publish(msg)
            
            

    def loadObject(self):
        self.liftToTargetPosition(RobotConfiguration.LIFT_PLATFORM_POSITION_MAX)

    def unLoadObject(self):
        self.liftToTargetPosition(RobotConfiguration.LIFT_PLATFORM_POSITION_MIN)


    def moveToNode(self, node, speed):
        goal = MapManager.instance().nodeToPos(node)   

        while not self.isRightFacingWay(goal):
            if self.stop_behavior == True:
                return

            if self.status != RobotStatus.Paused:
                self.turnRobot(goal)
            
                self.battery = self.battery - 0.001
                time.sleep(0.1)

        while not self.isRightPosition(goal):
            if self.stop_behavior == True:
                return
            
            self.moveStraight(speed)

            self.battery = self.battery - 0.001
            time.sleep(0.1)

    def moveBackToNode(self, node, speed):
        goal = MapManager.instance().nodeToPos(node)   

        while not self.isRightPosition(goal):
            if self.stop_behavior == True:
                return
            
            self.moveStraight(-speed)

            self.battery = self.battery - 0.001
            time.sleep(0.1)






    def isRightFacingWay(self, goal):
        diff_x = goal.x - self.position.x
        diff_y = goal.y - self.position.y

        angle_to_goal = math.atan2(diff_y, diff_x)
        angle_diff = self.theta - angle_to_goal

        if angle_diff > math.pi:
            angle_diff = angle_diff - 2.0 * math.pi
        elif angle_diff < -math.pi:
            angle_diff = angle_diff + 2.0 * math.pi

        if angle_diff > RobotConfiguration.ANGLE_THRESHOLD:
            return False
        elif angle_diff < - RobotConfiguration.ANGLE_THRESHOLD:
            return False
        
        return True



    def turnRobot(self, goal):
        diff_x = goal.x - self.position.x
        diff_y = goal.y - self.position.y

        angle_to_goal = math.atan2(diff_y, diff_x)
        angle_diff = self.theta - angle_to_goal

        if angle_diff > math.pi:
            angle_diff = angle_diff - 2.0 * math.pi
        elif angle_diff < -math.pi:
            angle_diff = angle_diff + 2.0 * math.pi
                    

        if angle_diff > RobotConfiguration.ANGLE_THRESHOLD:
            self.speed.linear.x = 0.0
            self.speed.angular.z = - min(1.0, abs(angle_diff) * 5)
            
            
        elif angle_diff < - RobotConfiguration.ANGLE_THRESHOLD:
            self.speed.linear.x = 0.0
            self.speed.angular.z = min(1.0, abs(angle_diff) * 5)

        self.speedPublisher.publish(self.speed)


    def moveStraight(self, speed):
        self.speed.linear.x = speed
        self.speed.angular.z = 0.0

        self.speedPublisher.publish(self.speed)








    
class RobotController():
    def __init__(self):
        self.adaptor = Adaptor(self)

        self.serviceQueue = list()

        self.robots = {
            RobotID.AMR_LIFT5: Robot(RobotID.AMR_LIFT5, self.adaptor),
            RobotID.AMR_LIFT6: Robot(RobotID.AMR_LIFT6, self.adaptor),
            RobotID.AMR_LIFT7: Robot(RobotID.AMR_LIFT7, self.adaptor),
            # "a": Robot("a", self.adaptor),
            # "b": Robot("b", self.adaptor),
            # "c": Robot("c", self.adaptor),
        }

        self.publisher = Publisher(self.adaptor, self.robots)
        self.runThread = threading.Thread(target=self.run)
        self.exit_event = threading.Event()  # 종료 이벤트

    def start(self):
        self.adaptor.start()
        self.publisher.start()
        self.runThread.start()

    def run(self):
        while not self.exit_event.is_set():
            time.sleep(0.15)
            if len(self.serviceQueue) > 0:
                serviceInstance = self.serviceQueue.pop()
                self.serviceReceived(serviceInstance)
                robot = self.robots.get(serviceInstance.robotName)
                if robot is not None:
                    robot.executeBehavior(serviceInstance)


    

    def serviceReceived(self, service):
        if isinstance(service, LoginService):
            ackMessage = MessageFactory.newAckMessage(service)
            self.adaptor.broadcast(ackMessage)
            service.setResult(ServiceResult.Success)
            self.serviceFinished(service)
        elif isinstance(service, RobotService):
            # print("RobotService")
            # self.robots[service.robotName].setService(service)
            ackMessage = MessageFactory.newAckMessage(service)
            self.adaptor.broadcast(ackMessage)
            
            # if service.result is not None:
                # self.serviceFinished(service)
        elif isinstance(service, PalletizerService):
            print("Palletizer msg")
            # if not (isinstance(service, EnterPalletizerService) or isinstance(service, ExitPalletizerService)):
            #     ackMessage = MessageFactory.newAckMessage(service)
            #     self.adaptor.broadcast(ackMessage)
            # if service.result is not None:
            #     self.serviceFinished(service)





if __name__ == '__main__':
    rospy.init_node('robotController')
    rc = RobotController()
    rc.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Ctrl-C detected, shutting down...")
        rc.exit_event.set() 
        for robot in rc.robots.values():
            robot.stop_behavior = True
            
    







