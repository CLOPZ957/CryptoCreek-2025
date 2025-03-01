
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

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.joystick = wpilib.XboxController(0)
        self.joystick1 = wpilib.XboxController(1)
        
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
        elevator_motors = wpilib.MotorControllerGroup(self.elevator_left, self.elevator_right)


        #End Effector
        self.EF_rightmotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)
        self.EF_leftmotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)


    def autonomousInit(self):
        return super().autonomousInit()

        

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
        #Algae Arm
        if (self.joystick.getRawButton(kLB)):
            self.algae_arm.set(-0.25)
        elif (self.joystick.getRawButtonReleased(kLB)):
            self.algae_arm.set(0)

        if (self.joystick.getRawButton(kRB)):
            self.algae_arm.set(0.25)
        elif (self.joystick.getRawButtonReleased(kRB)):
            self.algae_arm.set(0)
        
        if (self.joystick.getRawButton(kX)):
            self.algae_wheels.set(-0.25)
        elif (self.joystick.getRawButtonReleased(kX)):
            self.algae_wheels.set(0)
            
        
        if (self.joystick.getRawButton(kB)):
            self.algae_wheels.set(0.25)
        elif (self.joystick.getRawButtonReleased(kB)):
            self.algae_wheels.set(0)



        #Elevator
        if (self.joystick1.getPOV() == 0):
            self.elevator_right.set(0.2)
            self.elevator_left.set(-0.2)
        elif (self.joystick1.getPOV() == 180):
            self.elevator_right.set(-0.2)
            self.elevator_left.set(0.2)
        else:
            self.elevator_left.set(0)
            self.elevator_right.set(0)

        #End Effector
        if (self.joystick1.getRawButton(kY)):
            self.EF_leftmotor.set(-0.7)
            self.EF_rightmotor.set(0.9)
        elif (self.joystick1.getRawButtonReleased(kY)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)

        if (self.joystick1.getRawButton(kA)):
            self.EF_leftmotor.set(-0.5)
            self.EF_rightmotor.set(0.5)
        elif (self.joystick1.getRawButtonReleased(kA)):
            self.EF_leftmotor.set(0)
            self.EF_rightmotor.set(0)

        #drive motor
        # print(f"RightY: {RightY} - LeftY: {LeftY}")

        #exponential movement

        """Called when operation control mode is enabled"""

    # Exponential movement adjustment
        rightTrigger = self.joystick.getRightTriggerAxis()  
        leftTrigger = self.joystick.getLeftTriggerAxis()   
        LeftX = self.joystick.getLeftX()                     
        RightX = self.joystick.getRightX()
        print (LeftX)       

        if(rightTrigger > 0):
            rightTrigger = (rightTrigger**2)*-0.5
        else:
            rightTrigger = (rightTrigger**1)*0
        
        if(rightTrigger == 0 and leftTrigger > 0):
            leftTrigger = (leftTrigger**2)*.5
        else:
            leftTrigger = (leftTrigger**1)*0

        if (LeftX < 0):
            LeftX = (LeftX**4)*-.5
        elif (LeftX > 0):
            LeftX = (LeftX**4)*.5
        else:
            LeftX = (LeftX**4)

        #rightTrigger = rightTrigger *-0.5      
        #leftTrigger = leftTrigger *1.2


        self.drive.arcadeDrive(rightTrigger or leftTrigger, LeftX)

    
def disabledInit(self) -> None:
        # This just makes sure that our simulation code knows that the motor is off
        self.lf_motor.set(0)
        self.lr_motor.set(0)
        self.rf_motor.set(0)
        self.rr_motor.set(0)
        
