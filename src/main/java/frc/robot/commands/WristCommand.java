// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  
  private WristSubsystem WRIST_SUBSYSTEM; 
  private PIDController pidController; 
  

  double wristSetpoint; 
  double measurement; 
  double startTime; 
  double runTime; 

  boolean inAutonomousMode; 

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wrist, double setPoint, boolean inAuto, double rt) {
    this.WRIST_SUBSYSTEM = wrist; 
    this.pidController = new PIDController(WristConstants.wristKP, WristConstants.wristKI, WristConstants.wristKD); 
    this.wristSetpoint = setPoint; 
    this.inAutonomousMode = inAuto; 
    this.runTime = rt; 
    addRequirements(WRIST_SUBSYSTEM);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    startTime = System.currentTimeMillis(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = WRIST_SUBSYSTEM.getEncoder(); 
    double setValue = pidController.calculate(measurement, wristSetpoint); 
    
    if(setValue >= WristConstants.wristMaxSpeed){
      setValue = WristConstants.wristMaxSpeed; 
    }

    else if(setValue <= -WristConstants.wristMaxSpeed){
      setValue = -WristConstants.wristMaxSpeed; 
    }

    WRIST_SUBSYSTEM.setWrist(setValue * WristConstants.wristCutoff);

    // WRIST_SUBSYSTEM.setWrist(wristSetpoint);

    SmartDashboard.putNumber("wrist encoder: ", measurement);  
    SmartDashboard.putNumber("wrist motor speed", setValue); 

    SmartDashboard.putNumber("wrist set value", wristSetpoint);
    SmartDashboard.putNumber("wrist current", WRIST_SUBSYSTEM.getWristCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    WRIST_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(inAutonomousMode == true){

      if(Math.abs(startTime - System.currentTimeMillis()) < runTime){
        if(Math.abs(WRIST_SUBSYSTEM.getEncoder() - wristSetpoint) < WristConstants.wristTolerance){
          return true; 
        }
  
        else{
          return false; 
        } 
      }

      else{
        return true; 
      }

 
    }

    else{
      return false; 
    } 
  }
}
