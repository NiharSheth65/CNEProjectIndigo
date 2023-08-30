// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.Led2Subsystem;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand extends CommandBase {
  /** Creates a new LedCommand. */
  
  private LedSubsystem LED_SUBSYSTEM;  
  // private Led2Subsystem LED_2SUBSYSTEM; 

  int red, green, blue; 

  public LedCommand(LedSubsystem led, int r, int g, int b) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED_SUBSYSTEM = led; 
    // this.LED_2SUBSYSTEM = led2; 
    this.red = r; 
    this.green = g; 
    this.blue = b; 

    addRequirements(LED_SUBSYSTEM);
    // addRequirements(LED_2SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED_SUBSYSTEM.setOneColour(0, 0, 0);
    // LED_2SUBSYSTEM.setTwoColour(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LED_SUBSYSTEM.setOneColour(red, green, blue);
    // LED_2SUBSYSTEM.setTwoColour(red, green, blue);
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
