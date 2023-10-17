// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.limelightSubsystem;

public class visionDriveCommand extends CommandBase {
  
  private limelightSubsystem limelightSubsystem; 
  private DriveSubsystem driveSubsystem; 

  private PIDController driveVisionPID; 
  private PIDController turnVisionPID; 
  
  double gyroTargetPosition; 
  boolean cancelMode = false; 
  double distanceFromLimelightToGoalInches; 
  
  double tvValue; 
  int tvMissedCoutner; 
  double lastMeasuredValue; 
  double distanceLast; 

  int setPipelineNumber; 

  /** Creates a new visionDriveCommand. */
  public visionDriveCommand(DriveSubsystem drive, limelightSubsystem limelight, boolean cancel, int pipelineNumber) {
    this.limelightSubsystem = limelight; 
    this.driveSubsystem = drive; 
    // this.driveVisionPID = new PIDController(0.0175, 0, 0); 
    this.driveVisionPID = new PIDController(0.025, 0, 0); 
    this.turnVisionPID = new PIDController(0.0075, 0, 0); 
    this.cancelMode = cancel; 
    this.setPipelineNumber = pipelineNumber; 

    addRequirements(limelightSubsystem);
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveVisionPID.reset(); 
    turnVisionPID.reset();
    gyroTargetPosition = driveSubsystem.getYaw(); 
    tvMissedCoutner = 0; 
    limelightSubsystem.setPipeline(setPipelineNumber);
    limelightSubsystem.setLED(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double limelightLensHeight = 8.5; 
    double goalHeight = 16.5; 
    double limelightMountAngle = 20; 

    double angleToGoalDegrees = limelightMountAngle + limelightSubsystem.ty(); 
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees); 

    if(limelightSubsystem.tv() == 0){
      tvMissedCoutner++; 
    }

    
    else if(limelightSubsystem.tv() == 1){
      tvMissedCoutner = 0;  
      distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight)/Math.tan(angleToGoalRadians); 
      distanceLast = distanceFromLimelightToGoalInches; 
    }
    
    
    double driveSpeed = -1 * driveVisionPID.calculate(distanceFromLimelightToGoalInches, 15); 
    double turnSpeed = turnVisionPID.calculate(driveSubsystem.getYaw(), gyroTargetPosition); 

    if(driveSpeed > 0.75){
      driveSpeed = 0.75; 
    }

    else if(driveSpeed < -0.75){
      driveSpeed = -0.75; 
    }

    SmartDashboard.putNumber("distance to target", distanceFromLimelightToGoalInches); 
    SmartDashboard.putNumber("leftSpeed", driveSpeed + turnSpeed); 
    SmartDashboard.putNumber("right speed", driveSpeed - turnSpeed); 

    // driveSubsystem.tankMode(turnSpeed, -1*turnSpeed);
    driveSubsystem.tankMode(driveSpeed + turnSpeed, driveSpeed - turnSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(cancelMode == true){
      return true; 
    }

    else if(distanceFromLimelightToGoalInches < 22 && tvMissedCoutner == 0){
      return true; 
    }

    else{
      return false; 
    }
      
  }
}
