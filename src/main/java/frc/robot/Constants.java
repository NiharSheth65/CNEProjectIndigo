// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    public static final int primaryControllerPort = 0;
    public static final int secondaryControllerPort = 1;

    // button ports
    public static final int BUTTON_A_PORT = 1;
    public static final int BUTTON_B_PORT = 2;
    public static final int BUTTON_X_PORT = 3;
    public static final int BUTTON_Y_PORT = 4;
    public static final int BUTTON_RB_PORT = 6;
    public static final int BUTTON_LB_PORT = 5;

    public static final int BUTTON_START = 8;  
    public static final int BUTTON_RIGHT_JOYSTICK_PORT = 10; 
    public static final int BUTTON_LEFT_JOYSTICK_PORT = 9;   

    public static final int driveJoystickAxis = 1;
    public static final int turnJoystickAxis = 4;

    public static final int rightTriggerAxis = 3; 
    public static final double rightTriggerThreshold = 0.5; 

  }

  public static class DriverConstants{

    public static final int leftFrontID = 1; 
    public static final int leftBackID = 2; 
    public static final int rightFrontID = 3; 
    public static final int rightBackID = 4; 


    public static final double ksVolts = -0.22131; 
    public static final double kvVoltsSecondPerMeter = 3.4154;  
    public static final double kaVoltsSecondSquarePerMeter = 4.15; 
    public static final double kPDriveVelocity = 3.0239; 


    public static final double kTrackWidthMeters = Units.inchesToMeters(21); 
    public static final DifferentialDriveKinematics kDifferentialDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters); 
    
    public static final double kMaxSpeedMetersPerSecond = 3; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;  


    public static final double kRamseteB = 2; 
    public static final double kRamseteZeta = 0.7; 
    
    public static final double kGearRatio = 12.76; 
    public static final double kWheelRadiusInches = 3; 

    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1/(kGearRatio * 2*Math.PI*Units.inchesToMeters(kWheelRadiusInches))*10)); 
  
    public static final double driveSlew = 5; 
    public static final double turnSlew = 0.975; 

    public static final double driveDeadband = 0.1; 
    public static final double turnDeadband = 0.1; 
  
    public static final double driveSpeed = 0.8; 
    public static final double turnSpeed = 0.7;
  }

  public static class IntakeConstants{
    public static final int intakeMotorPort = 0; 
    public static final int switchOnePort = 0; 
    public static final int switchTwoPort = 1; 

    public static final double intakeSpeed = -0.7; 
    public static final double intakeOffSpeed = 0; 
    
    public static final double outtakeSpeed = 1;
    public static final double outtakeSlowSpeed = 0.25;

  }

  public static class LedConstants{
    public static final int ledPort = 1; 
    public static final int ledLength = 60; 
    
    public static final int[] greenColourCode = {0, 255, 0}; 
    public static final int[] indigoColourCode = {75, 0, 130}; 
  
  }

  public static class WristConstants{
    public static final int WristPort = 5; 
  
    // wrist PID
    public static final double wristKP = 0.1; 
    public static final double wristKI = 0; 
    public static final double wristKD = 0;

    public static final double wristTolerance = 0.15; 
    public static final double wristMaxSpeed = 1.0; 
  
    // wrist positions 
    public static final double wristIntakePosition = 43; 
    public static final double wristRestPosition = 2; 
    public static final double wristShootPosition = 14; 
    public static final double wristAboveIntakePosition = 37; 

    // wrist speed 
    public static final double wristCutoff = 0.6; 
  }

  public static class AutoConstants{
    public static final double clearDistance = -10;
    // public static final double dockDistance = -2.76;
    public static final double dockDistance = -2.76;
    public static final double autoDriveSpeed = -0.5;
  
    public static final double autoDockingSpeed = 0.25; 

    public static final double autoCutOff = 0.7;
  }

}
 