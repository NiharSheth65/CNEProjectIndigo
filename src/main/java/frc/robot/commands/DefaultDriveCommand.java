// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private Joystick joy; 

  double drive;
  double turn; 
  double driveSpeed;

  SlewRateLimiter drive_Limiter = new SlewRateLimiter(DriverConstants.driveSlew); 
  SlewRateLimiter turn_Limiter = new SlewRateLimiter(DriverConstants.turnSlew); 

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


    if(joy.getRawButton(OperatorConstants.BUTTON_RB_PORT)){
      driveSpeed = DriverConstants.driveFastSpeed; 
    }

    else{
      driveSpeed = DriverConstants.driveSlowSpeed;
    }

    if(Math.abs(joy.getRawAxis(OperatorConstants.driveJoystickAxis)) < DriverConstants.driveDeadband){
      drive = 0; 
    }
    else{
      drive = joy.getRawAxis(OperatorConstants.driveJoystickAxis)*-driveSpeed;  
    }

    // turn 
    if(Math.abs(joy.getRawAxis(OperatorConstants.turnJoystickAxis)) < DriverConstants.turnDeadband){
      turn = 0; 
    }

    else{
      // turn = turn_Limiter.calculate(joy.getRawAxis(4)); 
      turn = joy.getRawAxis(OperatorConstants.turnJoystickAxis)*-DriverConstants.turnSpeed; 
    }

    // drive = -joy.getRawAxis(1) * 0.7; 
    // turn = -joy.getRawAxis(4) * 0.25; 

    DRIVE_SUBSYSTEM.set(drive_Limiter.calculate(drive), turn);
  
    SmartDashboard.putNumber("drive speed", drive); 
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
