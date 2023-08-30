// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private VictorSP intakeMotor = new VictorSP(0); 
  private DigitalInput intakeSwitch1 = new DigitalInput(0); 
  private DigitalInput intakeSwitch2 = new DigitalInput(1); 

  public IntakeSubsystem() { 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Switch 1", intakeSwitch1.get()); 
    SmartDashboard.putBoolean("Intake Switch 2", intakeSwitch2.get()); 
  }

  public boolean intakeSwitchOneValue(){
    return intakeSwitch1.get(); 
  }

  public boolean intakeSwitchTwoValue(){
    return intakeSwitch2.get(); 
  }

  public void intake(double speed){
    intakeMotor.set(speed);
  }

  public void stop(){
    intakeMotor.set(0);
  }

}
