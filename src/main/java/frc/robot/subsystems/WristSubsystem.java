// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.WritableByteChannel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {

  // private CANSparkMax wristMotor; 
  private CANSparkMax wristMotor; 

  private RelativeEncoder wristEncoder; 

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    wristMotor = new CANSparkMax(5, MotorType.kBrushless);  
    wristMotor.restoreFactoryDefaults(); 
    wristEncoder = wristMotor.getEncoder();
    wristMotor.getEncoder().setPosition(0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getEncoder(){
    return wristEncoder.getPosition(); 
  }

  public void setWrist(double wristSpeed){
    wristMotor.set(wristSpeed);
  }

  public void stop(){
    wristMotor.set(0);
  }

}
