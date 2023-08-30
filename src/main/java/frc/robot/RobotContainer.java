// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LedCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.autoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.Led2Subsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  // private final Led2Subsystem m_Led2Subsystem = new Led2Subsystem(); 

  private final Joystick joystick = new Joystick(OperatorConstants.primaryControllerPort); 
  private final CommandGenericHID controller = new CommandGenericHID(0);  

  private final JoystickButton BUTTON_RB = new JoystickButton(joystick, OperatorConstants.BUTTON_RB_PORT); 
  private final JoystickButton BUTTON_LB = new JoystickButton(joystick, OperatorConstants.BUTTON_LB_PORT); 

  private final JoystickButton BUTTON_A = new JoystickButton(joystick, OperatorConstants.BUTTON_A_PORT);
  private final JoystickButton BUTTON_B = new JoystickButton(joystick, OperatorConstants.BUTTON_B_PORT); 
  private final JoystickButton BUTTON_X = new JoystickButton(joystick, OperatorConstants.BUTTON_X_PORT); 
  private final JoystickButton BUTTON_Y = new JoystickButton(joystick, OperatorConstants.BUTTON_Y_PORT); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.primaryControllerPort);



  private final Command m_shootCommand = new autoShootCommand(m_IntakeSubsystem); 
  
  SendableChooser<Command> m_autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CameraServer.startAutomaticCapture(); 

    // Configure the trigger bindings
    m_autoChooser.setDefaultOption("shoot command", m_shootCommand);
    m_autoChooser.addOption("also a shoot command", m_shootCommand);
    m_autoChooser.addOption("shoot command also", m_shootCommand);

    SmartDashboard.putData(m_autoChooser);
    
    configureBindings();
    defaultCommands();
  }

  private void defaultCommands(){
    m_driveSubsystem.setDefaultCommand(new DefaultDriveCommand(m_driveSubsystem, joystick));
    // m_LedSubsystem.setDefaultCommand(new LedCommand(m_LedSubsystem, 150, 0, 255));
    m_IntakeSubsystem.setDefaultCommand(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, 0));
    
    // m_WristSubsystem.setDefaultCommand(new WristCommand(m_WristSubsystem));
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
    BUTTON_RB.onTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, 1)); 
    BUTTON_RB.onFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, 0)); 

    // BUTTON_RB.onTrue(new OuttakeCommand(m_IntakeSubsystem, 1)); 
    // BUTTON_RB.onFalse(new OuttakeCommand(m_IntakeSubsystem, 0)); 
    
    BUTTON_LB.onTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem,-0.70)); 
    BUTTON_LB.onFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem,0)); 

    // BUTTON_A.toggleOnTrue(new WristCommand(m_WristSubsystem, 22.5)); 
    // BUTTON_A.onTrue(new WristCommand(m_WristSubsystem, -30)); 
    
    BUTTON_A.toggleOnTrue(
      Commands.race(
        new WristCommand(m_WristSubsystem, 22.5),
        new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, -0.5)
      ).andThen(
        new WristCommand(m_WristSubsystem, 7) 
      )
    ); 


    BUTTON_B.toggleOnTrue(new WristCommand(m_WristSubsystem, 1)); 
    // BUTTON_B.onTrue(new WristCommand(m_WristSubsystem, 30)); 
    
    BUTTON_X.toggleOnTrue(new WristCommand(m_WristSubsystem, 7)); 
    BUTTON_Y.toggleOnTrue(new WristCommand(m_WristSubsystem, 20)); 
    // BUTTON_X.onTrue(new WristCommand(m_WristSubsystem, 0)); 
    

    // controller.axisGreaterThan(3, 0.5).toggleOnTrue(new OuttakeCommand(m_IntakeSubsystem, 0.5)); 
    // controller.axisGreaterThan(3, 0.5).toggleOnFalse(new OuttakeCommand(m_IntakeSubsystem, 0)); 
   
    controller.axisGreaterThan(3, 0.5).toggleOnFalse(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, 0)); 
    controller.axisGreaterThan(3, 0.5).toggleOnTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, 0.5)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected(); 
  }

  public void autonomousInit(){
  }



}
