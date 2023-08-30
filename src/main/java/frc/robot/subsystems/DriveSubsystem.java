// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private CANSparkMax leftMotorFront; 
  private CANSparkMax leftMotorBack; 

  private CANSparkMax rightMotorFront; 
  private CANSparkMax rightMotorBack; 

  private RelativeEncoder rightFrontEncoder; 
  private RelativeEncoder leftFrontEncoder; 
  
  private AHRS navx; 
  
  public DriveSubsystem() {
    leftMotorFront = new CANSparkMax(1, MotorType.kBrushless); 
    leftMotorBack = new CANSparkMax(2, MotorType.kBrushless); 
    rightMotorFront = new CANSparkMax(3, MotorType.kBrushless); 
    rightMotorBack = new CANSparkMax(4, MotorType.kBrushless); 
  
    leftMotorFront.restoreFactoryDefaults(); 
    leftMotorBack.restoreFactoryDefaults(); 
    rightMotorFront.restoreFactoryDefaults(); 
    rightMotorBack.restoreFactoryDefaults(); 
  
    rightMotorFront.setInverted(true);
    rightMotorBack.setInverted(true);
  
    rightMotorBack.follow(rightMotorFront); 
    leftMotorBack.follow(leftMotorFront); 
    
    rightFrontEncoder = rightMotorFront.getEncoder(); 
    leftFrontEncoder = leftMotorFront.getEncoder(); 

    rightMotorFront.getEncoder().setPosition(0); 
    leftMotorFront.getEncoder().setPosition(0); 
  
    navx = new AHRS(SPI.Port.kMXP); 
    navx.reset();
    navx.zeroYaw();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double leftEncoder(){
    return leftFrontEncoder.getPosition(); 
  }

  public double rightEncoder(){
    return rightFrontEncoder.getPosition(); 
  }

  public double getYaw(){
    return navx.getYaw(); 
  }

  public double getRoll(){
    return navx.getRoll(); 
  }

  public double getPitch(){
    return navx.getPitch(); 
  }

  public void set(double drive, double turn){
    rightMotorFront.set(drive + turn);
    leftMotorFront.set(drive - turn);
  }

  public void stop(){
    rightMotorFront.stopMotor();
    leftMotorFront.stopMotor(); 
  }
}
