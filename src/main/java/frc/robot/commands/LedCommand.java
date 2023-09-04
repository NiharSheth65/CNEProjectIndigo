// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.Led2Subsystem;
import frc.robot.subsystems.LedSubsystem;

public class LedCommand extends CommandBase {
  /** Creates a new LedCommand. */
  
  private LedSubsystem LED_SUBSYSTEM;  
  private IntakeSubsystem INTAKE_SUBSYSTEM; 
  private DriveSubsystem DRIVE_SUBSYSTEM; 
  private Joystick joy; 
  // private Led2Subsystem LED_2SUBSYSTEM; 

  int red, green, blue; 

  double startTime; 

  public LedCommand(LedSubsystem led, IntakeSubsystem intake, DriveSubsystem drive, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED_SUBSYSTEM = led; 
    // this.LED_2SUBSYSTEM = led2; 
    this.INTAKE_SUBSYSTEM = intake; 
    this.DRIVE_SUBSYSTEM = drive; 
    this.joy = joystick; 

    addRequirements(LED_SUBSYSTEM);
    // addRequirements(LED_2SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED_SUBSYSTEM.setOneColour(0, 0, 0);
    red = 0; 
    green = 0; 
    blue = 0;

    startTime = System.currentTimeMillis();
    // LED_2SUBSYSTEM.setTwoColour(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(INTAKE_SUBSYSTEM.intakeSwitchOneValue() == true){
      red = LedConstants.greenColourCode[0];
      green = LedConstants.greenColourCode[1];
      blue = LedConstants.greenColourCode[2];

      // LED_SUBSYSTEM.setOneColour(red, green, blue);
    }


    else if(INTAKE_SUBSYSTEM.intakeSwitchOneValue() == false){
      
      if(joy.getRawButton(OperatorConstants.BUTTON_LB_PORT)){
        

        if(Math.abs(System.currentTimeMillis() - startTime) < LedConstants.policeFlashCycle){

          if(Math.abs(startTime - System.currentTimeMillis()) < LedConstants.policeFlashCycle/2){
            red = LedConstants.blueColourCode[0]; 
            green = LedConstants.blueColourCode[1]; 
            blue = LedConstants.blueColourCode[2]; 
          }
    
          else if(Math.abs(startTime - System.currentTimeMillis()) > LedConstants.policeFlashCycle/2){
            red = LedConstants.redColourCode[0]; 
            green = LedConstants.redColourCode[1]; 
            blue = LedConstants.redColourCode[2];
          }
        }

        else{
          startTime = System.currentTimeMillis(); 
        }

      }

      else{
        red = LedConstants.indigoColourCode[0]; 
        green = LedConstants.indigoColourCode[1]; 
        blue = LedConstants.indigoColourCode[2];

        
      }
      
    }

    LED_SUBSYSTEM.setOneColour(red, green, blue); 

    // else{
    //   LED_SUBSYSTEM.setOneColour(LedConstants.indigoColourCode[0], LedConstants.indigoColourCode[1], LedConstants.indigoColourCode[2]);
    // }
    
  
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
