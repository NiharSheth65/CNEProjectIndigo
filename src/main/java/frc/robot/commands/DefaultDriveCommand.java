// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private Joystick joy; 

  double drive;
  double turn; 

  SlewRateLimiter drive_Limiter = new SlewRateLimiter(0.9); 
  SlewRateLimiter turn_Limiter = new SlewRateLimiter(0.9); 

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DriveSubsystem drive, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 
    this.joy = joystick; 
    addRequirements(DRIVE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive = 0; 
    turn = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(joy.getRawAxis(1)) < 0.1){
      drive = 0; 
    }
    else{
      drive = joy.getRawAxis(1)*-0.70;  
    }

    // turn 
    if(Math.abs(joy.getRawAxis(4)) < 0.1){
      turn = 0; 
    }
    else{
      // turn = turn_Limiter.calculate(joy.getRawAxis(4)); 
      turn = joy.getRawAxis(4)*-0.25; 
    }

    // drive = -joy.getRawAxis(1) * 0.7; 
    // turn = -joy.getRawAxis(4) * 0.25; 
    

    SmartDashboard.putNumber("gyro yaw", DRIVE_SUBSYSTEM.getYaw()); 
    SmartDashboard.putNumber("gyro pitch", DRIVE_SUBSYSTEM.getPitch()); 
    SmartDashboard.putNumber("gyro roll", DRIVE_SUBSYSTEM.getRoll()); 


    SmartDashboard.putNumber("right encoder", DRIVE_SUBSYSTEM.rightEncoder()); 
    SmartDashboard.putNumber("left encoder", DRIVE_SUBSYSTEM.leftEncoder());
    SmartDashboard.putNumber("average disatance", (DRIVE_SUBSYSTEM.rightEncoder() + DRIVE_SUBSYSTEM.leftEncoder())/2); 

    DRIVE_SUBSYSTEM.set(drive_Limiter.calculate(drive), turn_Limiter.calculate(turn));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
