// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class autoGyroCommand extends CommandBase {
  /** Creates a new autoGyroCommand. */
  DriveSubsystem DRIVE_SUBSYSTEM;
  double gyroInitPosition; 
  double correctionSpeed; 

  public autoGyroCommand(DriveSubsystem drive, double startPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.gyroInitPosition = startPosition; 
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("initial gyro position", gyroInitPosition);
    correctionSpeed = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(gyroInitPosition - DRIVE_SUBSYSTEM.getRoll()) < AutoConstants.autoCutOff){
      correctionSpeed = 0;    
    }

    else if(DRIVE_SUBSYSTEM.getRoll() > gyroInitPosition){
      correctionSpeed = AutoConstants.autoDockingSpeedOverShoot; 
    }

    else if(DRIVE_SUBSYSTEM.getRoll() < gyroInitPosition){
      correctionSpeed = AutoConstants.autoDockingSpeedUnderShoot; 
    }

    DRIVE_SUBSYSTEM.set(correctionSpeed, 0);

    SmartDashboard.putNumber("gyro speed", correctionSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
    // if(Math.abs(gyroInitPosition - DRIVE_SUBSYSTEM.getHeading()) < 0.05){
    //   return true;
    
    // }

    // else{
    //   return false;
    // }
  }


}
