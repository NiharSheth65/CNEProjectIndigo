// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LedCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.autoShootAndClearCommand;
import frc.robot.commands.autoShootAndDockCommand;
import frc.robot.commands.buttonTurnCommand;
import frc.robot.commands.limelightReadCommand;
import frc.robot.commands.visionAlignCommand;
import frc.robot.commands.visionDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.Led2Subsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.limelightSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import javax.imageio.IIOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(); 
  private final WristSubsystem m_WristSubsystem = new WristSubsystem(); 
  private final LedSubsystem m_LedSubsystem = new LedSubsystem();   
  private final limelightSubsystem m_LimlightSubsystem = new limelightSubsystem(); 

  // private final Led2Subsystem m_Led2Subsystem = new Led2Subsystem(); 

  private final Joystick joystick = new Joystick(OperatorConstants.primaryControllerPort); 
  private final Joystick joystickSecondary = new Joystick(OperatorConstants.secondaryControllerPort); 

  // private final CommandGenericHID controllerPrimary = new CommandGenericHID(OperatorConstants.primaryControllerPort);  
  private final CommandGenericHID controllerSecondary = new CommandGenericHID(OperatorConstants.secondaryControllerPort);  

  private final JoystickButton BUTTON_RB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB_SECONDARY = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_LB_PORT); 

  private final JoystickButton BUTTON_RB = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 

  private final JoystickButton BUTTON_A_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT); 
  private final JoystickButton BUTTON_B_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_X_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_X_PORT); 
  private final JoystickButton BUTTON_Y_PRIMARY = new JoystickButton(joystick, OperatorConstants.BUTTON_Y_PORT); 
  
  private final JoystickButton BUTTON_A = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_A_PORT);
  private final JoystickButton BUTTON_B = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_X = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_X_PORT); 
  private final JoystickButton BUTTON_Y = new JoystickButton(joystickSecondary, OperatorConstants.BUTTON_Y_PORT); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.primaryControllerPort);

  private final Command m_shootAndDockCommand = new autoShootAndDockCommand(m_driveSubsystem, m_WristSubsystem, m_IntakeSubsystem, m_LedSubsystem); 
  private final Command m_shootAndClearCommand = new autoShootAndClearCommand(m_driveSubsystem, m_WristSubsystem, m_IntakeSubsystem, m_LedSubsystem); 

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CameraServer.startAutomaticCapture(); 

    // Configure the trigger bindings

    m_autoChooser.setDefaultOption("shoot and dock", m_shootAndDockCommand);
    m_autoChooser.addOption("shoot and clear", m_shootAndClearCommand);
    // m_autoChooser.addOption("curvy", loadPath(
    //   "src/main/deploy/pathplanner/deploy/pathplanner/generatedJSON/New Path.wpilib.json", true));

    // m_autoChooser.addOption("straight", loadPath(
    //   "C:/Users/NACI Robotics 1/Desktop/frc2023/cneBot/src/main/deploy/deploypathplanner/generatedJSON/New Path.wpilib.json", true));


    Shuffleboard.getTab("Autonomous").add(m_autoChooser);
  
    configureBindings();
    defaultCommands();
    
  }

  public Command loadPath(String filename, boolean resetOdometry){
    Trajectory trajectory; 

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename); 
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath); 
    }catch(IOException exception){
      DriverStation.reportError("unable to open trajectory" + filename, exception.getStackTrace());
      System.out.print("error with files");
      return new InstantCommand(); 
    }
    
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_driveSubsystem::getPose, new RamseteController(DriverConstants.kRamseteB, DriverConstants.kRamseteZeta), new SimpleMotorFeedforward(DriverConstants.ksVolts, DriverConstants.kvVoltsSecondPerMeter, DriverConstants.kaVoltsSecondSquarePerMeter), DriverConstants.kDifferentialDriveKinematics, m_driveSubsystem::getWheelSpeeds, new PIDController(DriverConstants.kPDriveVelocity, 0, 0), new PIDController(DriverConstants.kPDriveVelocity, 0, 0), m_driveSubsystem::tankDriveVolts, m_driveSubsystem); 
    
    if(resetOdometry){
      return new SequentialCommandGroup(
        new InstantCommand(() -> m_driveSubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand); 
    }else{
      return ramseteCommand; 
    }

  }

  private void defaultCommands(){
  
    m_LedSubsystem.setDefaultCommand(new LedCommand(m_LedSubsystem, m_IntakeSubsystem, m_driveSubsystem, joystick));
    m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(m_driveSubsystem, joystick));
    m_LimlightSubsystem.setDefaultCommand(new limelightReadCommand(m_LimlightSubsystem));

    // m_IntakeSubsystem.setDefaultCommand(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeOffSpeed, false));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Trigger teleopTrigger = new Trigger(() -> RobotState.isEnabled() && RobotState.isTeleop());
    // teleopTrigger.onTrue(new ResetEncoderCommand());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // BUTTON_RB_SECONDARY.onTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSpeed, false)); 
    
    BUTTON_RB_SECONDARY.onTrue(
        // new SequentialCommandGroup(
          // new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, false), 
          // new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, -0.6, false)
        // ) 
  
        new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSpeed, false)
    ); 

      // new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSpeed, false)); 
    
    
    BUTTON_RB_SECONDARY.onFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeOffSpeed, false)); 
    
    BUTTON_LB_SECONDARY.onTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeSpeed, false)); 
    BUTTON_LB_SECONDARY.onFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeOffSpeed, false)); 


    BUTTON_A_PRIMARY.onTrue(
      new buttonTurnCommand(m_driveSubsystem, 90, false) 
    ); 

    BUTTON_A_PRIMARY.onFalse(
      new buttonTurnCommand(m_driveSubsystem, 0, true) 
    ); 

    BUTTON_B_PRIMARY.onTrue(
      new buttonTurnCommand(m_driveSubsystem, -90, false) 
    ); 

    BUTTON_B_PRIMARY.onFalse(
      new buttonTurnCommand(m_driveSubsystem, 0, true)
    );

    BUTTON_Y_PRIMARY.onTrue(
      new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, false, 0)
      .andThen(new visionDriveCommand(m_driveSubsystem, m_LimlightSubsystem, false, 0))
      .andThen(new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, false, 0))
      .andThen(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSpeed, false))
    ); 

      // new limelightReadCommand(m_LimlightSubsystem, false) 
      

    BUTTON_Y_PRIMARY.onFalse(
      // new limelightReadCommand(m_LimlightSubsystem, true) 
    // new buttonTurnCommand(m_driveSubsystem, 0, true) 
      new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, true, 0)
      .andThen(new visionDriveCommand(m_driveSubsystem, m_LimlightSubsystem, true, 0))
    ); 

    BUTTON_X_PRIMARY.onTrue(
      new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, false, 1)
      .andThen(new visionDriveCommand(m_driveSubsystem, m_LimlightSubsystem, false, 1))
      .andThen(new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, false, 1))
      .andThen(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSpeed, false))
    ); 

      // new limelightReadCommand(m_LimlightSubsystem, false) 
      

    BUTTON_X_PRIMARY.onFalse(
      // new limelightReadCommand(m_LimlightSubsystem, true) 
    // new buttonTurnCommand(m_driveSubsystem, 0, true) 
      new visionAlignCommand(m_driveSubsystem, m_LimlightSubsystem, true, 1)
      .andThen(new visionDriveCommand(m_driveSubsystem, m_LimlightSubsystem, true, 1))
    ); 

    // BUTTON_A.toggleOnTrue(new WristCommand(m_WristSubsystem, WristConstants.wristIntakePosition, false, 0));
    
    BUTTON_A.toggleOnTrue(
      new ParallelRaceGroup(
        new WristCommand(m_WristSubsystem, WristConstants.wristIntakePosition, false, 0),
        new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeSpeed, false)
      )
    );

    BUTTON_A.toggleOnFalse(new WristCommand(m_WristSubsystem, WristConstants.wristShootPosition, false, 0)); 

    BUTTON_B.toggleOnTrue(new WristCommand(m_WristSubsystem, WristConstants.wristRestPosition, false, 0)); 
    
    BUTTON_X.toggleOnTrue(new WristCommand(m_WristSubsystem, WristConstants.wristShootPosition, false, 0)); 

    BUTTON_Y.toggleOnTrue(new WristCommand(m_WristSubsystem, WristConstants.wristAboveIntakePosition, false, 0)); 
   
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeOffSpeed, false)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeSlowSpeed, false)); 
    
    // controller.axisGreaterThan(OperatorConstants.rightTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(
    //   new SequentialCommandGroup(
    //     new WristCommand(m_WristSubsystem, WristConstants.wristLowerOuttakePosition, true, 0),
    //     new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeMidSpeed, false)
    //   )
    // );  

    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.intakeOffSpeed, false)); 
    controllerSecondary.axisGreaterThan(OperatorConstants.leftTriggerAxis, OperatorConstants.triggerThreshold).toggleOnTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, IntakeConstants.outtakeMidSpeed, false));   
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public DriveSubsystem getDriveSystem(){
    return m_driveSubsystem; 
  }

  public void resetEncoders(){
    m_driveSubsystem.resetEncoders();
    m_WristSubsystem.resetEncoders(); 
  }

  public Command getAutonomousCommand() { 
    autonomousInit();
    return m_autoChooser.getSelected(); 
  }

  public void autonomousInit(){
    m_driveSubsystem.resetEncoders();
    m_WristSubsystem.resetEncoders();
    // GEARBOX_SUBSYSTEM.speedMode();
  }
}

 // public void autonomousInit(){
 // }



