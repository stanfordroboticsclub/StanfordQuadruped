from logging import debug
from time import sleep
import sys
from pybullet_utils import bullet_client
import pybullet
use_drive = False
if use_drive:
  import pupper_drive
import json
import msgpack
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

PYBULLET_KP_GAIN = 4
PYBULLET_KD_GAIN = 0.2

linkNames = [   b"rightFrontLeg",
                b"rightFrontUpperLeg",
                b"rightFrontLowerLeg",
                b"leftFrontLeg",
                b"leftFrontUpperLeg",
                b"leftFrontLowerLeg",
                b"rightRearLeg",
                b"rightRearUpperLeg",
                b"rightRearLowerLeg",
                b"leftRearLeg",
                b"leftRearUpperLeg",
                b"leftRearLowerLeg",
            ]

class DummySerial:
    _read_line = ''

    def __init__(self,
                 port=None,
                 baudrate=9600,
                 bytesize=EIGHTBITS,
                 parity=PARITY_NONE,
                 stopbits=STOPBITS_ONE,
                 timeout=None,
                 xonxoff=False,
                 rtscts=False,
                 write_timeout=None,
                 dsrdtr=False,
                 inter_byte_timeout=None,
                 exclusive=None,
                 **kwargs
                 ):
        self._timeout = 0
        self._written = None
        self._line_term = '\r\n'
        self._rate = 0.01
        debug(f'Setup DummySerial write pusher')
        print("creating pupper drive system")
        if use_drive:
          self.drive = pupper_drive.DriveSystem()
        self.p = bullet_client.BulletClient()
        self.pupper_body_uid = -1
        self.pupper_link_indices=[]
        self.p.setTimeStep(0.001)
        #self.p.setPhysicsEngineParameter(numSubSteps=1)
        
        found_pupper_sim = False
        
        for i in range (self.p.getNumBodies()):
          b = self.p.getBodyUniqueId(i)
          print("body ", b, self.p.getBodyInfo(b))
          if self.p.getBodyInfo(b)[1]==b'pupper_v2_dji':
            print("found Pupper in sim")
            found_pupper_sim = True
            self.pupper_body_uid = b
            for linkName in linkNames:
              for link in range(self.p.getNumJoints(b)):
                #print("link:",link,"=",self.p.getJointInfo(b,link)[12])
                if self.p.getJointInfo(b,link)[12] == linkName:
                  print("found link!")
                  self.pupper_link_indices.append(link)
        print("self.pupper_link_indices=",self.pupper_link_indices)
        self.time_stamp = 0
        if not found_pupper_sim:
          sys.exit("Error: Cannot find pupper, is pupper_server running?")
        #self.p.configureDebugVisualizer(rgbBackground=[0,1,0])
        if use_drive:
          print("SetIdle")
          self.drive.SetIdle()
          print("SetMaxCurrent 2.")
          self.drive.SetMaxCurrent(2.0)
          self.drive.SetPositionKp(8.0)
          self.drive.SetPositionKd(2.0)
          self.drive.SetActivations([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
          #self.drive.SetActivations([1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
          self.drive.SetMaxCurrent(0.0)
        
        #self._mq_feedback = bind_socket(listen_feedback_push, zmq.PUSH)

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value

    @property
    def termination(self):
        return self._line_term

    @termination.setter
    def termination(self, value):
        self._line_term = value

    @property
    def readline_msg(self):
        data = {}
        #todo, implement
        data['ts'] = self.time_stamp
        pos = [1,2,3]
        joint_states = self.p.getJointStates(self.pupper_body_uid,self.pupper_link_indices)
        print("joint_states=",joint_states)
        data['pos'] = pos
        
        #{'ts': 14382421, 'pos': [-0.18985143303871155, 0.8387252688407898, -1.4362961053848267, 0.2225763499736786, 0.8596257567405701, -1.4191665649414062, -0.22404637932777405, 0.8246424794197083, -1.4191664457321167, 0.19057580828666687, 0.8719614744186401, -1.433973789215088], 'vel': [-0.0, -0.0, 0.0, -0.0, 0.0, -0.0, -0.0, -0.0, 0.0, -0.0, 0.0, -0.0], 'cur': [-0.12800000607967377, -0.1850000023841858, -0.08399999886751175, 0.49000000953674316, 0.34700000286102295, 0.2680000066757202, -0.25600001215934753, 0.21199999749660492, 0.3310000002384186, 0.09600000083446503, -0.05700000002980232, -0.017999999225139618], 'pref': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 'lcur': [-0.20297390222549438, -0.12319455295801163, -0.04769688844680786, 0.4445544481277466, 0.29117271304130554, 0.29456692934036255, -0.2934328019618988, 0.18119065463542938, 0.2210077941417694, 0.1260087639093399, -0.051187947392463684, -0.02925008349120617]}
        
        json_data = json.dumps(data)
        self._read_line = json_data
        return self._read_line

    @property
    def write_msg(self):
        return self._written

    @property
    def rate(self):
        return self._rate

    @rate.setter
    def rate(self, value):
        self._rate = value

    def write(self, msg: bytes):
        #print("len(msg)=",len(msg))
        #print("msg=",msg)#debug(f'Sleep for {self._rate} seconds')
        b = bytearray()
        b.extend(msg)
        l = b[1]
        del b[0]
        del b[0]
        if (l==len(b)):
          #print("len verified to be:",l)
          pass
        data =  msgpack.unpackb(b, raw=False)
        
        #print("data=",data)
        if use_drive:
          if "kp" in data:
            #print("setting position kp=",data["kp"])
            self.drive.SetPositionKp(data["kp"])
          if "kd" in data:
            #print("setting position kd=",data["kd"])
            self.drive.SetPositionKd(data["kd"])
          if "cart_kp" in data:
            #print("setting Cartesian kp", data["cart_kp"])
            self.drive.SetCartesianKp(data["cart_kp"])
          if "max_current" in data:
            #print("setting max_current:", data["max_current"])
            self.drive.SetMaxCurrent(data["max_current"])
          if "cart_kd" in data:
            #print("setting Cartesian kd", data["cart_kd"])
            self.drive.SetCartesianKd(data["cart_kd"])
          if "activations" in data:
            
            #print("Setting activations:", data["activations"])
            self.drive.SetActivations(data["activations"])
        
        if "pos" in data:
          #print("Setting joint positions:", data["pos"])
          action_repeat = 10
          max_torque = 4
          useArray = True
          kp = PYBULLET_KP_GAIN
          kd = PYBULLET_KD_GAIN
          if useArray:
            self.p.setJointMotorControlArray(
                      self.pupper_body_uid,
                      self.pupper_link_indices,
                      pybullet.PD_CONTROL,
                      targetPositions=data["pos"],
                      positionGains = [kp]*12,
                      velocityGains = [kd]*12,
                      forces=[max_torque]*12)
          else:
            for a in range(len(self.pupper_link_indices)):
                link_index = self.pupper_link_indices[a]
                self.p.setJointMotorControl2(
                      self.pupper_body_uid,link_index, 
                      pybullet.PD_CONTROL,
                      targetPosition=data["pos"][a],
                      positionGain = kp,
                      velocityGain = kd,
                      force=max_torque)
          for i in range (action_repeat):
            self.p.setGravity(0, 0, -10)            
            self.p.stepSimulation()
          sleep(0.001*action_repeat)
          
        if "cart_pos" in data:
          if use_drive:
            #print("Setting Cartesian positions:", data["cart_pos"])
            self.drive.SetCartesianPositions(data["cart_pos"])
            
            measured_positions = []
            measured_velocities = []
            
            for a in range(len(self.pupper_link_indices)):
              link_index = self.pupper_link_indices[a]
              js = self.p.getJointState(self.pupper_body_uid,link_index)
              measured_positions.append(js[0])
              measured_velocities.append(js[1])
              
            #joint_states = self.p.getJointStates(self.pupper_body_uid,self.pupper_link_indices)
            #print("joint_states=",joint_states)
            
            #for js in joint_states:
            #  measured_positions.append(js[0])
            #  measured_velocities.append(js[1])
            
            #print("measured_positions=",measured_positions)
            #print("measured_velocities=",measured_velocities)
            self.drive.SetMeasuredPositions(measured_positions)
            self.drive.SetMeasuredVelocities(measured_velocities)
            
            self.drive.Update()
            currents = self.drive.GetLastCommandedCurrents()
            #print("currents=",currents)
            
            action_repeat = 10
            for i in range (action_repeat):
              for a in range(len(self.pupper_link_indices)):
                  link_index = self.pupper_link_indices[a]
                  current = currents[a]
                  CURRENT_TO_TORQUE = .4
                  torque = current * CURRENT_TO_TORQUE
                  self.p.setJointMotorControl2(self.pupper_body_uid,link_index, pybullet.TORQUE_CONTROL, force=torque)

              self.p.setGravity(0, 0, -10)            
              self.p.stepSimulation()
              #sleep(0.001)
          else:
            print("Error: you need to enable use_drive for this mode")
            print("Get the python module from https://github.com/erwincoumans/DJIPupperTests")
            print("And in the root of that clone run: python3 setup.py install --user")
          
        #self._mq_feedback.send(msg)

    def readline(self):
        #debug(f'Sleep for {self._rate} seconds')
        #sleep(self._rate)
        return f'{self.readline_msg}{self.termination}'.encode()
