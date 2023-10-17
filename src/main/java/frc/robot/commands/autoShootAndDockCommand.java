// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class autoShootAndDockCommand extends SequentialCommandGroup {
  /** Creates a new autoShootAndDockCommand. */
  public autoShootAndDockCommand(DriveSubsystem drive, WristSubsystem wrist, IntakeSubsystem intake, LedSubsystem led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    double autoInitPosition = drive.getRoll(); 
    
    addCommands(
      // new SequentialCommandGroup(
      //   new WristCommand(wrist, 7, true), 
      //   new IntakeCommand(intake, led, 0.5, true)
      // )

      // new WristCommand(wrist, WristConstants.wristShootPosition, true, 2000),
      // new IntakeCommand(intake, led, IntakeConstants.outtakeSpeed, true),
      new autoDriveForwardCommand(drive, AutoConstants.dockDistance, AutoConstants.autoDriveSpeed), 
      // new autoDriveForwardCommand(drive, -AutoConstants.dockDistance, -AutoConstants.autoDriveSpeed), 
      // new DelayCommand(750),
      new autoGyroCommand(drive, autoInitPosition)

    );
  }
}
