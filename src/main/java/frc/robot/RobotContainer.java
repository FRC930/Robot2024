// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.SetPositionsCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TestIndexerCommand;
import frc.robot.commands.TestShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.MechanismViewer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.Results;

import java.util.Optional;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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

    private static final String CANBUS = "rio";

    //--DIO IDS--\\
    private static final int TURRET_ENCODER_DIO = 0;

    private static final double TURRET_OFFSET = 0.0;

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
        .withKP(12)//TODO: Configure ALL
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0.5)
        .withKS(0)
        .withKV(0);

    private final Slot0Configs shootingS0CSimulation = 
      new Slot0Configs()
        .withKP(1)//TODO: Configure ALL
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(0);

    private final Slot0Configs climbingS0C = 
      new Slot0Configs()
        .withKP(0)//TODO: Configure ALL
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(0);

    private final Slot0Configs pivotS0C =
      new Slot0Configs()
        .withKP(0) 
        .withKI(0) 
        .withKD(0) 
        .withKA(0) 
        .withKG(0) 
        .withKS(0) 
        .withKV(0);

    private final ProfiledPIDController turretPID = new ProfiledPIDController(0.26, 0, 0, new Constraints(0, 0)); //TODO: Set good vals

    // ks overcomes friction on the turret
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(0.375, 0, 0); 

    
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
        ? new ElevatorIORobot(14, 15, CANBUS, shootingS0C, shootingMMC, ElevatorType.SHOOTING_ELEVATOR)
        : new ElevatorIOSim(14, 15, CANBUS, shootingS0CSimulation, shootingMMC, ElevatorType.SHOOTING_ELEVATOR));

    public final ElevatorSubsystem m_climbingElevatorSubsystem = new ElevatorSubsystem(
      Robot.isReal()
        ? new ElevatorIORobot(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR)
        : new ElevatorIOSim(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR));

    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(
      Robot.isReal()
        ? new PivotIORobot(5, CANBUS, 1, pivotS0C, pivotMMC)
        : new PivotIOSim(5, CANBUS, 1, pivotS0C, pivotMMC));

    // TODO: Figure out real motor and encoder id
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem(
      Robot.isReal()
        ? new TurretIORobot(6, TURRET_ENCODER_DIO, CANBUS, TURRET_OFFSET)
        : new TurretIOSim(6, TURRET_ENCODER_DIO, CANBUS, TURRET_OFFSET), 
        turretPID, turretFF);

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(3, CANBUS) : new RollerMotorIOSim(3, CANBUS),
        Robot.isReal() ? new RollerMotorIORobot(4, CANBUS) : new RollerMotorIOSim(4, CANBUS));

    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(20, CANBUS) : new RollerMotorIOSim(20, CANBUS),
        Robot.isReal() ? new TimeOfFlightIORobot(3, 200) : new TimeOfFlightIOSim(3));

    // TODO: Figure out real motor/ToF ids
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(19, CANBUS) : new RollerMotorIOSim(19, CANBUS),
        Robot.isReal() ? new RollerMotorIORobot(7, CANBUS) : new RollerMotorIOSim(7, CANBUS),
        Robot.isReal() ? new TimeOfFlightIORobot(1, 200) : new TimeOfFlightIOSim(1),
        Robot.isReal() ? new TimeOfFlightIORobot(2, 200) : new TimeOfFlightIOSim(2));

    MechanismViewer m_mechViewer = new MechanismViewer(m_pivotSubsystem, m_shootingElevatorSubsystem, m_climbingElevatorSubsystem, m_turretSubsystem);
    
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

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureTestBindings();
    // configureBindings();
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
    SmartDashboard.putNumber("KrakenLeftMotor", 0.0);
    SmartDashboard.putNumber("KrakenRightMotor", 0.0);
    SmartDashboard.putNumber("IndexerSetSpeed", 0.0);
    

    // SmartDashboard.putNumber("LeftSparkMotor", 0.0);
    // SmartDashboard.putNumber("RightSparkMotor", 0.0);

    //#region Default commands
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(negateBasedOnAlliance(-m_driverController.getLeftY() * MaxSpeed * PERCENT_SPEED)) // Drive forward with
                                                                                              // negative Y (forward)
                .withVelocityY(negateBasedOnAlliance(-m_driverController.getLeftX() * MaxSpeed * PERCENT_SPEED)) // Drive left with negative X (left)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                .withDeadband(JOYSTICK_DEADBAND)
                .withRotationalDeadband(JOYSTICK_ROTATIONAL_DEADBAND)
            // ).ignoringDisable(true)); // TODO CAUSED ISSUES with jumping driving during characterization
            ));

    m_shootingElevatorSubsystem.setDefaultCommand(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, 0.0));
          
    m_driverController.rightBumper().whileTrue(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, 2.0));
    
    //#region Button controls

    // m_driverController.y().whileTrue(new TestShooterCommand(m_shooterSubsystem));
    m_driverController.y().whileTrue(new TestShooterCommand(m_shooterSubsystem));
    
    m_driverController.x().whileTrue(new TestIndexerCommand(m_indexerSubsystem));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    //#endregion
    
    //#region POV controls
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
    //#endregion

    //#region Trigger/Bumper controls
    // reset the field-centric heading on left bumper press TODO test
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    m_driverController.leftTrigger().whileTrue(new LimeLightIntakeCommand(drivetrain, m_GamePieceUtility, new Pose2d(1.0, 0.0, new Rotation2d(0.0))));
    
    //#endregion 

    drivetrain.registerTelemetry(logger::telemeterize);

    }
  
  private void configureTestBindings() {
    SmartDashboard.putNumber("TurretSetPosition", 0.0);
    SmartDashboard.putNumber("ShooterLeftMotor", 0.0);
    SmartDashboard.putNumber("ShooterRightMotor", 0.0);
    SmartDashboard.putNumber("IndexerMotor", 0.0);

    m_intakeSubsystem.setDefaultCommand(
      new RunIntakeCommand(m_intakeSubsystem, -.15)
    );

    // m_turretSubsystem.setDefaultCommand(new InstantCommand(() -> m_turretSubsystem.setSpeed(m_driverController.getLeftX() / 4),m_turretSubsystem));

    m_driverController.b().whileTrue(new SetTurretPositionCommand(m_turretSubsystem, SmartDashboard.getNumber("TurretSetPosition", 0.0)));

    m_driverController.rightTrigger().whileTrue(
      new RunIntakeCommand(m_intakeSubsystem,0.6)
      .alongWith(new IndexerCommand(m_indexerSubsystem,SmartDashboard.getNumber("IndexerMotor",0.0)))
      .until(() -> m_indexerSubsystem.getSensor())); // Ends intake when note is detected in indexer
  
    m_driverController.leftTrigger().whileTrue(new ShooterCommand(m_shooterSubsystem,SmartDashboard.getNumber("ShooterLeftMotor", 0.0)/100,SmartDashboard.getNumber("ShooterRightMotor", 0.0)/100));
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

  public void robotPeriodic() {
    updateAllVision();
    m_mechViewer.periodic();
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

  public void simulationPeriodic() {
    // mechanismSimulator.periodic(); // Moved to robotPeriodic()
  }

}
