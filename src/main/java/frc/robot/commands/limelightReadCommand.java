// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelightSubsystem;

public class limelightReadCommand extends CommandBase {
  /** Creates a new limelightReadCommand. */

  private limelightSubsystem limelight;  

  public limelightReadCommand(limelightSubsystem limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight; 
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLED(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); 

    double tx = limelight.tx(); 
    double ty = limelight.ty(); 
    double ta = limelight.ta(); 
    double tv = limelight.tv(); 

    // double angleInDegrees = -ty; 
    // double distanceToAprilTag = 32.5; 

    // double angleInRadians = Math.toRadians(angleInDegrees); 
    // double travelDistance = distanceToAprilTag / Math.tan(angleInRadians); 

    double limelightLensHeight = 8.5; 
    double goalHeight = 16.5; 
    double limelightMountAngle = 20; 

    double angleToGoalDegrees = limelightMountAngle + ty; 
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees); 

    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight)/Math.tan(angleToGoalRadians); 

    SmartDashboard.putNumber("tx value", tx); 
    SmartDashboard.putNumber("ty value", ty); 
    SmartDashboard.putNumber("ta value", ta); 
    SmartDashboard.putNumber("tv value", tv); 
    SmartDashboard.putNumber("distance to tag", distanceFromLimelightToGoalInches); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if(mode){
      // return true; 
    // }
    // else{
      // return false;
    // }

    return false; 
  }
}
