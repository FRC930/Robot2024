// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoCommandManager;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.commands.SwerveLockCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CANLauncher;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.RobotInformation;
import frc.robot.utilities.RobotInformation.WhichRobot;
import frc.robot.utilities.SwerveModuleConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Alliance alliance = Alliance.Invalid;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;   
  private final int rotationAxis = XboxController.Axis.kRightX.value;


    // Which Robot code should we use? competition or not
    //Cannot use an ID of 0
    //Changed the turningMotorID and cancoderID from 0 to 3
    //https://buildmedia.readthedocs.org/media/pdf/phoenix-documentation/latest/phoenix-documentation.pdf
    //page 100
    RobotInformation robotInfo = 
        // Non-Competition robot attributes
        new RobotInformation(WhichRobot.PRACTICE_ROBOT,
          new SwerveModuleConstants(8, 9, 9, 288.721), 
          new SwerveModuleConstants(11, 10, 10, 58.184), 
          new SwerveModuleConstants(1, 3, 3, 268.770),
          new SwerveModuleConstants(18, 19, 19, 188.350)); 

  public static final int kDriverControllerPort = 0;
  public static final int kCodriverControllerPort = 1;
   
 /* Modules */
  public final SwerveModuleConstants frontLeftModule = robotInfo.getFrontLeft();
  public final SwerveModuleConstants frontRightModule =  robotInfo.getFrontRight();
  public final SwerveModuleConstants backLeftModule = robotInfo.getBackLeft();
  public final SwerveModuleConstants backRightModule = robotInfo.getBackRight();
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  CommandXboxController m_codriverController = new CommandXboxController(kCodriverControllerPort);

  // Subsystems \\
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  private final TeleopSwerve m_TeleopSwerve = new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true);
  private final CANLauncher m_launcher = new CANLauncher();

  
  // Commands \\
  private final SwerveLockCommand m_SwerveLockCommand = new SwerveLockCommand(m_robotDrive, true);

    
  private AutoCommandManager m_autoManager;
  private Map<String, Command> eventCommandMap = new HashMap<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // eventCommandMap = new HashMap<>();

    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.initCommands(eventCommandMap);

    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(m_TeleopSwerve);
    m_fieldSim.initSim();
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
 
  private void configureButtonBindings() {

    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/DriverController", DriverStation.getJoystickIsXbox(0));
    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/Co-DriverController", DriverStation.getJoystickIsXbox(1));
    
    //Final Button Bindings
    //--DRIVER CONTROLLER--//
    // Set the default command for the drivetrain to drive using the joysticks
    m_robotDrive.setDefaultCommand(new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true));

    /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
     * command for 1 seconds and then run the LaunchNote command */
    m_driverController
        .rightBumper()
        .whileTrue(
            new PrepareLaunch(m_launcher)
                .withTimeout(PrepareLaunch.getKLauncherDelay())
                .andThen(new LaunchNote(m_launcher))
                .handleInterrupt(() -> m_launcher.stop()));

    // Set up a binding to run the intake command while the operator is pressing and holding the
    // left Bumper
     m_driverController.leftBumper().whileTrue(m_launcher.getIntakeCommand());

    // Slow drive
     m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_TeleopSwerve.toggleSpeed()));
  
    //Lock wheels in an X position
     m_driverController.x().toggleOnTrue(m_SwerveLockCommand);

    
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Method to run before teleop starts, needed to help reset April Tag direction before teleop if operator does not do 
   * autonomous first.
   */
  public void teleopInit() {
    refollowAllMotors();
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public void disabledInit() {
  }

  public void testInit() {
    stopSubsystems();
    refollowAllMotors();
  }

  public void testPeriodic() {
  }

  public void testExit() {
    refollowAllMotors();
  }

  private void refollowAllMotors() {
  }

  private void stopSubsystems() {
  }

}