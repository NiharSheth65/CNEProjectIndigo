// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class buttonTurnCommand extends CommandBase {
  /** Creates a new buttonTurnCommand. */
  DriveSubsystem DRIVE_SUBSYSTEM; 
  private PIDController buttonPID; 


  double gyroInitAngle; 
  double gyroSetpoint; 
  boolean turnBoolean; 
  

  public buttonTurnCommand(DriveSubsystem drive, double angle, boolean turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.gyroSetpoint = angle; 
    this.turnBoolean = turn; 
    this.buttonPID = new PIDController(0.00775, 0.00415, 0); 
    
    addRequirements(DRIVE_SUBSYSTEM);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DRIVE_SUBSYSTEM.zeroHeading();
    gyroInitAngle = DRIVE_SUBSYSTEM.getYaw(); 
    buttonPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double target = gyroInitAngle + gyroSetpoint; 
    
    double outPutSpeed = buttonPID.calculate(DRIVE_SUBSYSTEM.getYaw(), target); 

    if(outPutSpeed > 0.8){
      outPutSpeed = 0.8;
    }

    else if(outPutSpeed < -0.8){
      outPutSpeed = -0.8; 
    }

    boolean inPlace = false;

    if(Math.abs(target - DRIVE_SUBSYSTEM.getYaw()) < 0.5){
      inPlace = true; 
    }

    else{
      inPlace = false; 
    }

    SmartDashboard.putBoolean("turnComple", inPlace);
    SmartDashboard.putNumber("output speed", outPutSpeed); 
    SmartDashboard.putNumber("init angle", gyroInitAngle); 
    SmartDashboard.putNumber("degrees travelled", DRIVE_SUBSYSTEM.getYaw() - gyroInitAngle);

    DRIVE_SUBSYSTEM.tankMode(outPutSpeed, -outPutSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turnBoolean == false){
      return false; 
    }
    else{
      return true; 
    }
  }
}
