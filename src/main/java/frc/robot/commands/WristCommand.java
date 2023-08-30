// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  
  private WristSubsystem WRIST_SUBSYSTEM; 
  private PIDController pidController; 
  

  double wristSetpoint; 
  double measurement; 

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wrist, double setPoint) {
    this.WRIST_SUBSYSTEM = wrist; 
    this.pidController = new PIDController(0.04, 0, 0); 
    this.wristSetpoint = setPoint; 
    addRequirements(WRIST_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = WRIST_SUBSYSTEM.getEncoder(); 
    double setValue = pidController.calculate(measurement, wristSetpoint); 
    
    if(setValue >= 1.0){
      setValue = 1.0; 
    }

    else if(setValue <= -1.0){
      setValue = -1.0; 
    }

    WRIST_SUBSYSTEM.setWrist(setValue);
    
    // WRIST_SUBSYSTEM.setWrist(wristSetpoint);

    SmartDashboard.putNumber("wrist encoder: ", measurement);  
    SmartDashboard.putNumber("motor speed", setValue); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    WRIST_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(WRIST_SUBSYSTEM.getEncoder() - wristSetpoint) < 0.2){
    //   return true; 
    // }

    // else{
    //   return false; 
    // }  

    return false; 
  }
}
