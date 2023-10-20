// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private CANSparkMax leftMotorFront = new CANSparkMax(DriverConstants.leftFrontID, MotorType.kBrushless); 
  private CANSparkMax leftMotorBack  = new CANSparkMax(DriverConstants.leftBackID, MotorType.kBrushless); 
  private CANSparkMax rightMotorFront =  new CANSparkMax(DriverConstants.rightFrontID, MotorType.kBrushless); 
  private CANSparkMax rightMotorBack = new CANSparkMax(DriverConstants.rightBackID, MotorType.kBrushless); 

  private RelativeEncoder rightFrontEncoder = rightMotorFront.getEncoder(); 
  private RelativeEncoder leftFrontEncoder = leftMotorFront.getEncoder(); 

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftMotorFront, leftMotorBack); 
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightMotorFront, rightMotorBack); 
  

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup); 
  
  // private final static Gyro navx = new AHRS(SPI.Port.kMXP); 
  private AHRS navx;

  private final DifferentialDriveOdometry m_Odometry;  

  public DriveSubsystem() {
  
    leftMotorFront.restoreFactoryDefaults(); 
    leftMotorBack.restoreFactoryDefaults(); 
    rightMotorFront.restoreFactoryDefaults(); 
    rightMotorBack.restoreFactoryDefaults(); 

    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);
 
    rightMotorBack.follow(rightMotorFront); 
    leftMotorBack.follow(leftMotorFront); 
  
    rightFrontEncoder.setPosition(0); 
    leftFrontEncoder.setPosition(0); 

    rightFrontEncoder.setPositionConversionFactor(DriverConstants.kLinearDistanceConversionFactor); 
    leftFrontEncoder.setPositionConversionFactor(DriverConstants.kLinearDistanceConversionFactor); 
    
    rightFrontEncoder.setVelocityConversionFactor(DriverConstants.kLinearDistanceConversionFactor/60); 
    leftFrontEncoder.setVelocityConversionFactor(DriverConstants.kLinearDistanceConversionFactor/60); 

    // navx.reset();
    // navx.calibrate(); 
    // resetEncoders(); 

    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();
    navx.zeroYaw();
    navx.calibrate(); 

    m_Odometry = new DifferentialDriveOdometry(navx.getRotation2d(), 0, 0);
    m_Odometry.resetPosition(new Rotation2d(), leftEncoderPosition(), rightEncoderPosition() , new Pose2d());
    // m_Odometry
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(navx.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());    
  
    // SmartDashboard.putNumber("gyro yaw", DRIVE_SUBSYSTEM.getYaw()); 
    // SmartDashboard.putNumber("gyro pitch", DRIVE_SUBSYSTEM.getPitch()); 
    // SmartDashboard.putNumber("gyro roll", DRIVE_SUBSYSTEM.getRoll()); 

    // SmartDashboard.putNumber("right encoder", rightEncoderPosition()); 
    // SmartDashboard.putNumber("left encoder", leftEncoderPosition());
    // SmartDashboard.putNumber("gyro ", getHeading());  
    SmartDashboard.putNumber("gyro PITCH", getPitch());   
    SmartDashboard.putNumber("gyro Yaw", getYaw());   
    SmartDashboard.putNumber("gyro Roll", getRoll());   
  }

  public void setBrakeMode(){
    leftMotorBack.setIdleMode(IdleMode.kBrake); 
    leftMotorFront.setIdleMode(IdleMode.kBrake); 
    rightMotorBack.setIdleMode(IdleMode.kBrake); 
    rightMotorFront.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoastMode(){
    leftMotorBack.setIdleMode(IdleMode.kCoast); 
    leftMotorFront.setIdleMode(IdleMode.kCoast); 
    rightMotorBack.setIdleMode(IdleMode.kCoast); 
    rightMotorFront.setIdleMode(IdleMode.kCoast); 
  }

  public void resetEncoders(){
    rightFrontEncoder.setPosition(0); 
    leftFrontEncoder.setPosition(0); 
  }

  public double leftEncoderPosition(){
    return leftFrontEncoder.getPosition(); 
  }

  public double rightEncoderPosition(){
    return -rightFrontEncoder.getPosition(); 
  }

  public double leftEncoderVelocity(){
    return leftFrontEncoder.getVelocity(); 
  }

  public double rightEncoderVelocity(){
    return -rightFrontEncoder.getVelocity(); 
  }

  public double getTurnRate(){
    return -navx.getRate(); 
  }

  // public static double getHeading(){
  //   return navx.getRotation2d().getDegrees(); 
  // }

  public double getRoll(){
    return navx.getRoll();
  }

  public double getPitch(){
    return navx.getPitch();
  }

  public double getYaw(){
    return navx.getYaw();
  }


  public Pose2d getPose(){
    return m_Odometry.getPoseMeters(); 
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders(); 
    // m_Odometry.resetPosition(navx.getRotation2d(), leftEncoderPosition(), rightEncoderPosition(), pose);
    m_Odometry.resetPosition(null, navx.getYaw(), getAverageEncoderDistance(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity(), rightEncoderVelocity()); 
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public RelativeEncoder getLeftEncoder(){
    return leftFrontEncoder; 
  }

  public RelativeEncoder getRightEncoder(){
    return rightFrontEncoder; 
  }

  public void setMaxOutput(double maxOutput){
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    navx.calibrate();
    navx.reset(); 
  }

  public Gyro getGryo(){
    return navx; 
  }
  
  public double gyroYaw(){
    return navx.getAngle();
  }

  public double getAverageEncoderDistance(){
    return ((leftEncoderPosition() + rightEncoderPosition())/2); 
  }

  // public double getYaw(){
  //   return navx.getYaw(); 
  // }

  // public double getRoll(){
  //   return navx.getRoll(); 
  // }

  // public double getPitch(){
  //   return navx.getPitch(); 
  // }


  public void set(double drive, double turn){
    // rightMotorFront.set(drive + turn);
    // leftMotorFront.set(drive - turn);
    differentialDrive.arcadeDrive(drive, turn);
  }

  public void tankMode(double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop(){
    // rightMotorFront.stopMotor();
    // leftMotorFront.stopMotor(); 
    differentialDrive.stopMotor();
  }
}
