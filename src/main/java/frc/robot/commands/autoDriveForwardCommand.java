// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class autoDriveForwardCommand extends CommandBase {
  /** Creates a new autoDriveForwardCommand. */

  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private PIDController gryoDrivePIDControllers; 

  private PIDController velocityRightPIDControllers; 
  private PIDController velocityLeftPIDControllers; 

  double gyroTargetPosition; 

  double driveForwardSetPoint; 
  double driveForwardSpeed; 

  double autonomousSpeed;
  double rightWheelOutput;
  double leftWheelOutput;

  double averageDistance; 

  public autoDriveForwardCommand(DriveSubsystem drive, double driveDistance, double driveSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVE_SUBSYSTEM = drive; 

    this.gryoDrivePIDControllers = new PIDController(0.005, 0, 0); 
    this.velocityLeftPIDControllers = new PIDController(1,0.0001,0); 
    this.velocityRightPIDControllers = new PIDController(1,0.0001,0);
    this.driveForwardSetPoint = driveDistance; 
    this.driveForwardSpeed = driveSpeed; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DRIVE_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroTargetPosition = DRIVE_SUBSYSTEM.getTurnRate(); 
    averageDistance = 0; 
    velocityLeftPIDControllers.reset();
    velocityRightPIDControllers.reset();
    gryoDrivePIDControllers.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftWheelVelocity = DRIVE_SUBSYSTEM.leftEncoderPosition(); 
    double rightWheelVelocity = DRIVE_SUBSYSTEM.rightEncoderPosition(); 
  

    double gyroSpeed = gryoDrivePIDControllers.calculate(DRIVE_SUBSYSTEM.getTurnRate(), gyroTargetPosition);
    // averageDistance = (DRIVE_SUBSYSTEM.getRightEncoder() + DRIVE_SUBSYSTEM.getLeftEncoder())/2; 
    averageDistance = DRIVE_SUBSYSTEM.getAverageEncoderDistance();  

    // sign should otherwise be flipped to >
    if (averageDistance > driveForwardSetPoint) {
      if (driveForwardSetPoint < 0) {
        autonomousSpeed = -1 * driveForwardSpeed;
      }

      else {
        // autonomousSpeed = 1000;
        autonomousSpeed = driveForwardSpeed;
      }
    }

    else {
      autonomousSpeed = 0.5;
      gyroSpeed = 0.5;
    }

    rightWheelOutput = velocityRightPIDControllers.calculate(rightWheelVelocity, autonomousSpeed);
    leftWheelOutput = velocityLeftPIDControllers.calculate(leftWheelVelocity, autonomousSpeed);

    double maxOutput = 0.8;

    if (leftWheelOutput > maxOutput) {
      leftWheelOutput = maxOutput;
    }

    else if (leftWheelOutput < -maxOutput) {
      leftWheelOutput = -maxOutput;
    }

    if (rightWheelOutput > maxOutput) {
      rightWheelOutput = maxOutput;
    }

    else if (rightWheelOutput < -maxOutput) {
      leftWheelOutput = -maxOutput;
    }


    // adding some gyro stuff
    // DRIVE_SUBSYSTEM.setRight(-1*(rightWheelOutput - gyroSpeed));
    // DRIVE_SUBSYSTEM.setLeft(-1*(leftWheelOutput + gyroSpeed));

    SmartDashboard.putNumber("set point`", driveForwardSetPoint); 
    SmartDashboard.putNumber("left wheel velocity", leftWheelVelocity); 
    DRIVE_SUBSYSTEM.tankMode(leftWheelVelocity, rightWheelVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
