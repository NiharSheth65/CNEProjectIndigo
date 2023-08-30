// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSubsystem Intake_Subsystem; 
  private LedSubsystem Led_Subsystem; 

  double intakeSpeed; 

  public IntakeCommand(IntakeSubsystem intake, LedSubsystem led, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Intake_Subsystem = intake; 
    this.intakeSpeed = speed; 
    this.Led_Subsystem = led; 
    addRequirements(Intake_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake_Subsystem.intake(intakeSpeed);

    if(Intake_Subsystem.intakeSwitchOneValue() == false || Intake_Subsystem.intakeSwitchTwoValue() == false){
      Led_Subsystem.setOneColour(0, 255, 0);
    }

    else{
      Led_Subsystem.setOneColour(150, 0, 255);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake_Subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // return false; 
    if(intakeSpeed < 0){
      if(Intake_Subsystem.intakeSwitchOneValue() == false ||Intake_Subsystem.intakeSwitchTwoValue() == false){
        return true; 
      }
  
      else{
        return false; 
      }
    }

    else{
      return false; 
    }


  }
}
