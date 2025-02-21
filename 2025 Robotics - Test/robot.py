
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
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
from wpilib import DriverStation


    
kY = 4 #Y Button
kX = 3 #X Button
kB = 2 #B Button
kA = 1 #A Button
kLB = 5 #LB Button
kRB = 6 #RB Button
kBack = 7 #Back Button
kStart = 8 #Start Button
kRT = 9 #Right Trigger Button
    
class DriveSubsystem(Subsystem):
    def __init__(self):
        
        
        config = RobotConfig.fromGUISettings()
        
        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPLTVController(0.02), # PPLTVController is the built in path following controller for differential drive trains
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
class RobotContainer:
    def getAutonomousCommand():
        # This method loads the auto when it is called, however, it is recommended
        # to first load your paths/autos when code starts, then return the
        # pre-loaded auto/path
        return PathPlannerAuto('Test Auto')    

class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.joystick = wpilib.XboxController(0)
        
        self.lf_motor = wpilib.PWMSparkMax(4) 
        self.lr_motor = wpilib.PWMSparkMax(3)
        self.rf_motor = wpilib.PWMSparkMax(9)
        self.rr_motor = wpilib.PWMSparkMax(2)

        l_motor = wpilib.MotorControllerGroup(self.lf_motor, self.lr_motor)
        r_motor = wpilib.MotorControllerGroup(self.rf_motor, self.rr_motor)

        l_motor.setInverted(True)
        

        self.drive = wpilib.drive.DifferentialDrive(l_motor, r_motor)
    
        algae_arm = rev.SparkMax()
        algae_wheels = rev.SparkMax()
        elevator = rev.SparkMax()
        EF_rightmotor = rev.SparkMax()
        EF_leftmotor = rev.SparkMax()


    def autonomousInit(self):
        return super().autonomousInit()

        

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
         #TEST THIS LATER (MOVEMENT ADJUSTMENTS)
        #drive motors
        rightTrigger = self.joystick.getRightTriggerAxis()    #New (test) code
        #RightY = self.joystick.getRightY()    #original code
        LeftX = self.joystick.getLeftX()
        

        RightX = self.joystick.getRightX()
        leftTrigger = self.joystick.getLeftTriggerAxis()
        # print(f"RightY: {RightY} - LeftY: {Left-Y}")
        
        if(rightTrigger > 0):
            rightTrigger = (rightTrigger**-4)*-1
        else:
            rightTrigger = (rightTrigger**4)

        if(leftTrigger > 0):
            rightTrigger = (rightTrigger**0)
        else:
            rightTrigger = (rightTrigger**4)*-1

        if(rightTrigger == 0):
            leftTrigger = (leftTrigger**4)*-1
        else:
            leftTrigger = (leftTrigger**4)*1
        
        


        rightTrigger = rightTrigger *1.5        
        leftTrigger = leftTrigger *.9  


        self.drive.arcadeDrive(rightTrigger, LeftX, leftTrigger)
    
        #Algae Movement
        # if (self.joystick.getRawButton())
            
        
        #self.drive.arcadeDrive(rightTrigger, LeftX, leftTrigger) # new arcade input

    def disabledInit(self) -> None:
        # This just makes sure that our simulation code knows that the motor is off
        self.lf_motor.set(0)
        self.lr_motor.set(0)
        self.rf_motor.set(0)
        self.rr_motor.set(0)
        
