
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive
import rev
import time
from wpilib import DriverStation
import wpilib
from wpilib.cameraserver import CameraServer
from ntcore import NetworkTableInstance
from wpimath.controller import PIDController


kY = 4 #Y Button
kX = 3 #X Button
kB = 2 #B Button
kA = 1 #A Button
kLB = 5 #LB Button
kRB = 6 #RB Button
kBack = 7 #Back Button
kStart = 8 #Start Button
    
class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    current_motion = 0
    current_z_rotation = 0

    def robotInit(self):
        # Auto 0 - No drive, 
        # Auto 1 - Drive Forward and trough

        self.preferredAuto = 1        
        self.joystick = wpilib.PS5Controller(0)
        self.joystick1= wpilib.XboxController(1)

        self.cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection")
        wpilib.CameraServer.launch("vision.py:main")
        
        
        self.lf_motor = rev.SparkMax(9, rev.SparkLowLevel.MotorType.kBrushless) 
        self.lr_motor = rev.SparkMax(2, rev.SparkLowLevel.MotorType.kBrushless)
        self.rf_motor = rev.SparkMax(4, rev.SparkLowLevel.MotorType.kBrushless)
        self.rr_motor = rev.SparkMax(3, rev.SparkLowLevel.MotorType.kBrushless)

        self.l_motor = wpilib.MotorControllerGroup(self.lf_motor, self.lr_motor)
        self.r_motor = wpilib.MotorControllerGroup(self.rf_motor, self.rr_motor)

        self.l_motor.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)
    
        #Algae
        self.algae_arm = rev.SparkMax(7, rev.SparkLowLevel.MotorType.kBrushless)
        self.algae_wheels = rev.SparkMax(8, rev.SparkLowLevel.MotorType.kBrushless)
       
        #Elevator
        self.elevator_left = rev.SparkMax(6, rev.SparkLowLevel.MotorType.kBrushless)
        self.elevator_right = rev.SparkMax(5, rev.SparkLowLevel.MotorType.kBrushless)

        self.elevatorLeftConfig = rev.SparkMaxConfig()
        self.elevatorLeftConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast)
        

        self.elevator_left.configure(self.elevatorLeftConfig, 
                rev.SparkBase.ResetMode.kResetSafeParameters, 
                rev.SparkBase.PersistMode.kPersistParameters)
        
        #new
        self.elevator_left.getEncoder()

        self.elevatorRightConfig = rev.SparkMaxConfig()
        self.elevatorRightConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast)
        self.elevatorRightConfig.follow(self.elevator_left, True)

        self.elevator_right.configure(self.elevatorRightConfig, 
                rev.SparkBase.ResetMode.kResetSafeParameters, 
                rev.SparkBase.PersistMode.kPersistParameters)
        
        self.elevatorClosedLoop = self.elevator_left.getClosedLoopController()
        self.elevatorClosedLoopConfig = rev.ClosedLoopConfig()
        self.elevatorClosedLoopConfig.pid(0.1,0.0,0.0,rev.ClosedLoopSlot.kSlot0)

        #End Effector
        self.EF_rightmotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)
        self.EF_leftmotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)


    def autonomousInit(self):
        self.timer = wpilib.Timer()
        self.timer.start()
        self.lastAction = 0

    def autonomousPeriodic(self):
         # AUTO 0 #
        if self.preferredAuto == 0:
            self.drive.arcadeDrive(0,0)

        if self.preferredAuto == 1:
            # step 1: drive forward
            if not self.timer.hasElapsed(1.5):
                self.drive.arcadeDrive(0.4, 0)
            else:
                self.drive.arcadeDrive(0,0)
                
            if self.timer.hasElapsed(4.5):
                self.EF_leftmotor.set(-0.8)
                self.EF_rightmotor.set(1)
            elif self.timer.hasElapsed(9):
                self.EF_leftmotor.set(0)
                self.EF_rightmotor.set(0)
            #Dont know how to stop the motors yet, hopefully this elif statement will work, if not, try another if statement
            #I think it will work bcause if a certain number of seconds have elapsed, then the motors should stop.
                

    def teleopPeriodic(self):
        if self.joystick1.getRawButtonPressed(kB):
                print("Setting camera 2")
                self.cameraSelection.setString("USB Camera 1")
        elif self.joystick1.getRawButtonReleased(kB):
                print("Setting camera 1")
                self.cameraSelection.setString("USB Camera 0")


        print(self.elevator_right.getEncoder().getPosition())
        
        #Algae Arm
        if (self.joystick.getRawButton(kRB)):
            self.algae_arm.set(-0.2)
        elif (self.joystick.getRawButtonReleased(kRB)):
            self.algae_arm.set(0)

        if (self.joystick.getRawButton(kLB)):
            self.algae_arm.set(0.2)
        elif (self.joystick.getRawButtonReleased(kLB)):
            self.algae_arm.set(0)
        
        if (self.joystick.getRawButton(kX)):
            self.algae_wheels.set(-0.25)
        elif (self.joystick.getRawButtonReleased(kX)):
            self.algae_wheels.set(0)
            
        
        if (self.joystick.getRawButton(kB)):
            self.algae_wheels.set(0.25)
        elif (self.joystick.getRawButtonReleased(kB)):
            self.algae_wheels.set(0)

        #Usual Speed is 0.54

        #Elevator
        if (self.joystick1.getPOV() == 0):
            self.elevator_right.set(0.54)
        elif (self.joystick1.getPOV() == 180):
            self.elevator_right.set(-0.54)
            self.elevator_left.set(0.54)
        else:
            self.elevator_left.set(0)
            self.elevator_right.set(0)

        #Level 1
        if self.joystick1.getRawButton(kA):
            self.EF_leftmotor.set(-0.7)
            self.EF_rightmotor.set(0.9)
        elif (self.joystick1.getRawButtonReleased(kA)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)

        #Level 2

        if self.joystick1.getRawButton(kX):
            start = time.time()
            while (time.time() < start + 0.34):
                self.elevator_left.set(-0.54)
        elif (self.joystick1.getRawButtonReleased(kX)):
            self.elevator_left.set(0)
            self.elevator_right.set(0)


        #Level 3
        if self.joystick1.getRawButton(kB):
            start = time.time()
            while (time.time() < start + 0.87):
                self.elevator_left.set(-0.54)
        elif (self.joystick1.getRawButtonReleased(kB)):
            self.elevator_left.set(0)
            self.elevator_right.set(0)
        
        #Level 4
        if self.joystick1.getRawButton(kY):
            start = time.time()
            while (time.time() < start + 1.84):
                self.elevator_left.set(-0.54)
        elif (self.joystick1.getRawButtonReleased(kY)):
            self.elevator_left.set(0)
            self.elevator_right.set(0)

        
        #End EFfector Shoot
        if self.joystick1.getRawButtonPressed(kRB):
            self.EF_leftmotor.set(-0.7)
            self.EF_rightmotor.set(0.7)
        elif (self.joystick1.getRawButtonReleased(kRB)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)

        if self.joystick1.getRawButtonPressed(kLB):
            self.EF_leftmotor.set(0.1)
            self.EF_rightmotor.set(-0.1)
        elif (self.joystick1.getRawButtonReleased(kLB)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)
        

        
        '''
        #End Effector
        if (self.joystick1.getRawButton(kA)):
            self.EF_leftmotor.set(-0.7)
            self.EF_rightmotor.set(0.9)
        elif (self.joystick1.getRawButtonReleased(kA)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)

        if (self.joystick1.getRawButton(kY)):
            self.EF_leftmotor.set(-0.5)
            self.EF_rightmotor.set(0.5)
        elif (self.joystick1.getRawButtonReleased(kY)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)
        
        #LAST YEAR CODE
        
        RightY = self.joystick.getRightY()
        LeftY = self.joystick.getLeftY()
        # print(f"RightY: {RightY} - LeftY: {LeftY}")
        
        #exponential movement
        if(RightY < 0):
            RightY = (RightY**2)*-1
        else:
            RightY = (RightY**2)
        
        if(LeftY < 0):
            LeftY = (LeftY**2)*-1
        else:
            LeftY = (LeftY**2)
     
        #this makes it turn slower
        if((RightY < 0.1 and RightY > -0.1) and (LeftY >= 0.5 or LeftY <= -0.5)):
            LeftY = LeftY * 0.66
        elif((LeftY < 0.1 and LeftY > -0.1) and (RightY >= 0.5 or RightY <=-0.5)):
            RightY = RightY * 0.66
        print(f"RightY: {RightY} - LeftY: {LeftY}")
        
        self.drive.tankDrive(LeftY, RightY)
        
        
'''
        #drive motors
        #exponential movement  
        #NEW TEST CODE

        
        forward = self.joystick.getR2Button()  
        backward = self.joystick.getL2Button()   
        zrotation = self.joystick.getLeftX()                     

        
        motion = 0
        speed_factor = 0.6
        rotation_factor = 0.5

        if (forward and backward):
            motion = 0
        elif (forward):
            motion = forward 
        elif (backward):
            #backward needs to be negated
            motion = -backward 
        
        
        motion *= speed_factor
        zrotation *= rotation_factor

        self.drive.arcadeDrive(motion, zrotation)
        
        '''
        self.variable_update(motion, zrotation)
        
        #Old TEST CODE
        if(forward > 0):
            forward = (forward**2)*-0.5
        else:
            forward = (forward**1)*0
        
        if(forward == 0 and backward > 0):
            backward = (backward**2)*.5
        else:
            backward = (backward**1)*0

        if (zrotation < 0):
            zrotation = (zrotation**4)*-.5
        elif (zrotation > 0):
            zrotation = (zrotation**4)*.5
        else:
            zrotation = (zrotation**4)
        
        #forward = forward *-0.5      
        #backward = backward *1.2


        self.drive.arcadeDrive(forward or backward, zrotation)
        
        # FOR NEW TEST CODE
    def variable_update(self,target_motion, target_zrotation)-> None:
        number_of_steps = 2950
        
        motion_step_width = (target_motion - self.current_motion) / number_of_steps
        zrotation_step_width = (target_zrotation - self.current_z_rotation) / number_of_steps

        for i in range(number_of_steps):
            motion = self.current_motion + motion_step_width
            zrotation = self.current_z_rotation + zrotation_step_width

            self.drive.arcadeDrive(motion, zrotation)

            self.current_motion = motion
            self.current_z_rotation = zrotation
    '''
def disabledInit(self) -> None:
        
        # This just makes sure that our simulation code knows that the motor is off
        self.lf_motor.set(0)
        self.lr_motor.set(0)
        self.rf_motor.set(0)
        self.rr_motor.set(0)
        
