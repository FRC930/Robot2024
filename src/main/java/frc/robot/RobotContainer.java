// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.IOs.TalonPosIO;
import frc.robot.IOs.TalonRollerEncoderIO;
import frc.robot.LimelightHelpers.Results;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.commands.SparkTestShooterCommand;
import frc.robot.commands.TestShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.MechanismSimulator;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SparkMaxShooterSubsystem;
import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorIORobot;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorType;
import frc.robot.subsystems.pivot.PivotIORobot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.roller.RollerMotorIORobot;
import frc.robot.subsystems.roller.RollerMotorIOSim;
import frc.robot.subsystems.timeofflight.TimeOfFlightIORobot;
import frc.robot.subsystems.timeofflight.TimeOfFlightIOSim;
import frc.robot.subsystems.turret.TurretIORobot;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.GamePieceDetectionUtility;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TestIndexerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final boolean UseLimeLightAprilTag = false; 

    private static final double   POV_PERCENT_SPEED = 0.3;
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1;
    private static final double PERCENT_SPEED = 0.3;

    private static final String RIO = "rio";

    private GamePieceDetectionUtility m_GamePieceUtility = new GamePieceDetectionUtility("limelight-front");

    // MK3 Falcon 13.6 ft/s 8.16:1 or 16.2 ft/s 6.86:1
    // https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=31575980703857
    final double MaxSpeed = Units.feetToMeters(16.2); //13.6); //  meters per second desired top speed
    final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        //TODO LOOK AT Generated version -- .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-cen

    //--PID AND FF CONSTANTS--\\
    private final Slot0Configs shootingS0C = 
      new Slot0Configs()
        .withKP(1)//TODO: Configure ALL
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(1);

    private final Slot0Configs climbingS0C = 
      new Slot0Configs()
        .withKP(1)//TODO: Configure ALL
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(1);

    private final Slot0Configs pivotS0C =
      new Slot0Configs()
        .withKP(1) 
        .withKI(0) 
        .withKD(0) 
        .withKA(1) 
        .withKG(0) 
        .withKS(0) 
        .withKV(1);

    //--MOTION MAGIC CONSTANTS--\\
    
    private final MotionMagicConfigs shootingMMC = 
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(5)
        .withMotionMagicExpo_kV(1)
        .withMotionMagicExpo_kA(4);
    
    private final MotionMagicConfigs climbingMMC = 
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(5)
        .withMotionMagicExpo_kV(1)
        .withMotionMagicExpo_kA(4);

    private final MotionMagicConfigs pivotMMC =
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(80)
        .withMotionMagicExpo_kV(1)
        .withMotionMagicExpo_kA(4);

    //--SUBSYSTEMS--\\

    public final ElevatorSubsystem m_shootingElevatorSubsystem = new ElevatorSubsystem(
        Robot.isReal()
        ? new ElevatorIORobot(12, 13, RIO, shootingS0C, shootingMMC, ElevatorType.SHOOTING_ELEVATOR)
        : new ElevatorIOSim(12, 13, RIO, shootingS0C, shootingMMC, ElevatorType.SHOOTING_ELEVATOR));

    public final ElevatorSubsystem m_climbingElevatorSubsystem = new ElevatorSubsystem(
      Robot.isReal()
        ? new ElevatorIORobot(14, 15, RIO, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR)
        : new ElevatorIOSim(14, 15, RIO, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR));

    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(
      Robot.isReal()
        ? new PivotIORobot(16, RIO, 1, pivotS0C, pivotMMC)
        : new PivotIOSim(16, RIO, 1, pivotS0C, pivotMMC));

    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem(
      Robot.isReal()
        ? new TurretIORobot(40, 3, RIO)
        : new TurretIOSim(40, 3, RIO));

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
      Robot.isReal() ? new RollerMotorIORobot(3, RIO) : new RollerMotorIOSim(3, RIO),
      Robot.isReal() ? new RollerMotorIORobot(4, RIO) : new RollerMotorIOSim(4, RIO));

    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(20, RIO) : new RollerMotorIOSim(20, RIO),
        Robot.isReal() ? new TimeOfFlightIORobot(0, 200) : new TimeOfFlightIOSim(0));

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(41, RIO) : new RollerMotorIOSim(41, RIO),
        Robot.isReal() ? new RollerMotorIORobot(42, RIO) : new RollerMotorIOSim(42, RIO),
        Robot.isReal() ? new TimeOfFlightIORobot(1, 200) : new TimeOfFlightIOSim(1),
        Robot.isReal() ? new TimeOfFlightIORobot(2, 200) : new TimeOfFlightIOSim(2));

    MechanismSimulator mechanismSimulator = new MechanismSimulator(m_pivotSubsystem, m_shootingElevatorSubsystem, m_climbingElevatorSubsystem, m_turretSubsystem);
    
    SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(MaxSpeed);
    
    private AutoCommandManager m_autoManager = new AutoCommandManager(drivetrain, m_GamePieceUtility);

    SwerveRequest.Idle idle = new SwerveRequest.Idle();

  // The robot's subsystems and commands are defined here...
  
  // private final SparkMaxShooterSubsystem m_sparkShooterSubsystem = new SparkMaxShooterSubsystem(3, 4);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    portForwardCameras();
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
    // SmartDashboard.putNumber("KrakenLeftMotor", 0.0);
    // SmartDashboard.putNumber("KrakenRightMotor", 0.0);

    SmartDashboard.putNumber("LeftSparkMotor", 0.0);
    SmartDashboard.putNumber("RightSparkMotor", 0.0);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(negateBasedOnAlliance(-m_driverController.getLeftY() * MaxSpeed * PERCENT_SPEED)) // Drive forward with
                                                                                              // negative Y (forward)
                .withVelocityY(negateBasedOnAlliance(-m_driverController.getLeftX() * MaxSpeed * PERCENT_SPEED)) // Drive left with negative X (left)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                .withDeadband(JOYSTICK_DEADBAND)
                .withRotationalDeadband(JOYSTICK_ROTATIONAL_DEADBAND)
            // ).ignoringDisable(true)); // TODO CAUSED ISSUES with jumping driving during characterization
            ));

    m_driverController.pov(0).whileTrue(
      drivetrain.applyRequest(() -> forwardStraight.withVelocityX(POV_PERCENT_SPEED * MaxSpeed).withVelocityY(0.0)
      ));
    m_driverController.pov(180).whileTrue(
      drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-POV_PERCENT_SPEED * MaxSpeed).withVelocityY(0.0)
      ));
    m_driverController.pov(90).whileTrue(
      drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(-POV_PERCENT_SPEED * MaxSpeed)
      ));
    m_driverController.pov(270).whileTrue(
      drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(POV_PERCENT_SPEED * MaxSpeed)
      ));

    // m_driverController.y().whileTrue(new TestShooterCommand(m_shooterSubsystem));

    // m_driverController.x().whileTrue(new TestIndexerCommand(m_indexerSubsystem));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    
    

    m_driverController.x().toggleOnTrue(m_pivotSubsystem.getTestCommand());

    m_driverController.b().toggleOnTrue(m_shootingElevatorSubsystem.getTestCommand());

    m_driverController.rightBumper().toggleOnTrue(m_climbingElevatorSubsystem.getTestCommand());
    

    // reset the field-centric heading on left bumper press TODO test
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    m_driverController.leftTrigger().whileTrue(new LimeLightIntakeCommand(drivetrain, m_GamePieceUtility, new Pose2d(1.0, 0.0, new Rotation2d(0.0))));

    drivetrain.registerTelemetry(logger::telemeterize);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoManager.getAutoManagerSelected();
  }

  public void periodic() {
    mechanismSimulator.periodic();
  }

  /*
   * Sets the port offsets
   */
  public void portForwardCameras() {
    // portForwardLimelight("front", 0);
    // portForwardLimelight("side", 10);

    portForwardLimelight("10.99.90.12", 0);
    portForwardLimelight("10.99.90.11", 10);
  }

  /**
   * Update all vision
   */
  public void updateAllVision() {
    if (UseLimeLightAprilTag) {  
      updateVisionOdometry("front");
      updateVisionOdometry("side");
    }
  }

  /**
   * Prints limelight, and limelight name. If the last result was valid, and the length is bigger than 0.
   * If there is a alliance to get the alliance, and if its red it sets the alliance to red; otherwise it sets the alliance to blue.
   * @param limeLightName
   */
  public void updateVisionOdometry(String limeLightName) {
      boolean useResult = true;
      Results lastResult = LimelightHelpers.getLatestResults("limelight-" + limeLightName).targetingResults;
      if (lastResult.valid && lastResult.targets_Fiducials.length > 0 && lastResult.targets_Fiducials[0].fiducialID != 0) {
          if (lastResult.targets_Fiducials.length == 1) {
              if (LimelightHelpers.getTA("limelight-" + limeLightName) > 0.27) { //The robot must be close to use only one April Tag at a time
                useResult = true;
              } else {
                useResult = false;
              }
          } else {
              useResult = true;
          }

          if (useResult) { //Always update odometry through blue alliance because blue origin is always (0,0)
              drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()); 
          }
      }
  }

  /** 
   * This method makes a port for the limelights
   * @param limeLightName the name of the Limelights
   * @param portOffset the offset needed to ensure that the ports for the cameras are not the same
   */
  public void portForwardLimelight(String limeLightName, int portOffset) {
      for (int limeLightPort = 5800; limeLightPort <= 5807; limeLightPort++) {
          int pcPort = limeLightPort + portOffset;
          // PortForwarder.add(pcPort, "limelight-" + limeLightName, limeLightPort);
          PortForwarder.add(pcPort, limeLightName, limeLightPort);
      }
  }
  /**
   * Checks whether alliance is red or blue so that teleop has correct facing controls IE: negate joystick value
   * 
   * @param joystickValue
   * @return negative or positive coordinate values depending on what alliance robot is on
   */
  public double negateBasedOnAlliance(double joystickValue) {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    //if on red alliance, return as negative
    //if on blue alliance, return as positive
    if (optionalAlliance.isPresent()){
      Alliance alliance = optionalAlliance.get();
      if (alliance == Alliance.Red) {
         return joystickValue*-1;
      }
    }
    return joystickValue;
  }

}
