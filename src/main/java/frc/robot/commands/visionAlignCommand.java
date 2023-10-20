// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.OutputStream;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.limelightSubsystem;

public class visionAlignCommand extends CommandBase {
  
  private PIDController visionPID; 

  private limelightSubsystem limelightSubsystem; 
  private DriveSubsystem driveSubsystem; 

  boolean turnBool; 
  double gyroValue; 
  double error; 

  double tvValue; 
  int tvMissedCoutner; 
  double lastMeasuredValue; 

  int setPipelineNumber; 

  double  kp; 
  double ki; 
  double kd; 

  int targetToTrack; 

  /** Creates a new visionAlignCommand. */
  public visionAlignCommand(DriveSubsystem drive, limelightSubsystem limelight, boolean turn, int pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = drive; 
    this.limelightSubsystem = limelight;
    // this.visionPID = new PIDController(0.0075, 0.025, 0); 
    // this.visionPID = new PIDController(0.01, 0.025, 0); 

    if(pipeline == 0){
      kp = 0.0175; 
      ki = 0.025; 
      kd = 0;   
    }

    else if(pipeline == 1){
      kp = 0.03; 
      ki = 0.025; 
      kd = 0;
    }


    this.visionPID = new PIDController(kp, ki, kd); 
    this.turnBool = turn; 
    this.setPipelineNumber = pipeline; 

    addRequirements(driveSubsystem);
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionPID.reset();
    tvMissedCoutner = 0;
    limelightSubsystem.setPipeline(setPipelineNumber);
    limelightSubsystem.setLED(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // int numberOfTargets = limelightSubsystem.numberOfTargets(); 
    int numberOfTargets = (int) limelightSubsystem.limelighTableRead().getEntry("tv").getDouble(0); 

    if(numberOfTargets > 0){
      double bestTargetArea = 0.0; 
      targetToTrack = 0; 

      for(int i=0; i < numberOfTargets; i++){
        double targetArea = limelightSubsystem.targetArea(i); 

        if(targetArea > bestTargetArea){
          bestTargetArea = targetArea; 
          targetToTrack = i; 
        }
      }

    }
   
    

    double targetValue = 0; 

    double measuredValue = limelightSubsystem.tx(); 
    tvValue = limelightSubsystem.tv(); 
    
    if(tvValue == 0){
      tvMissedCoutner++;
    }

    else{
      tvMissedCoutner = 0;
      lastMeasuredValue = limelightSubsystem.tx(); 
    }

    double outputSpeed = visionPID.calculate(measuredValue, targetValue); 

    if(outputSpeed > 0.8){
      outputSpeed = 0.8; 
    }

    else if(outputSpeed < -0.8){
      outputSpeed = -0.8; 
    }

    driveSubsystem.tankMode(-outputSpeed, outputSpeed);

    error = measuredValue - targetValue; 
    
    SmartDashboard.putNumber("align error", error); 
    SmartDashboard.putNumber("align speed", outputSpeed); 
    SmartDashboard.putNumber("number of targets", numberOfTargets); 
    boolean complete = false; 

    if(Math.abs(error) < 0.25){
      complete = true; 
      // turnBool = true; 
    }
    else{
      complete = false;
    }

    SmartDashboard.putBoolean("alignment", complete); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(turnBool == true){
      return true; 
    }
    
    else if(Math.abs(error) < 0.25 && tvMissedCoutner == 0){
      return true; 
    }
    
    else{
      return false; 
    }
  }
}
