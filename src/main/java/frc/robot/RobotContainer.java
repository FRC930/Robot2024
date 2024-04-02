// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.IOs.TalonPosIO;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurretAimCommand;
import frc.robot.commands.TurretRefineCommand;
import frc.robot.commands.tests.IndexerCommandTest;
import frc.robot.commands.tests.IntakeCommandTest;
import frc.robot.commands.tests.SetElevatorPositionCommandTest;
import frc.robot.commands.tests.SetPivotPositionCommandTest;
import frc.robot.commands.tests.SetTurretPositionCommandTest;
import frc.robot.commands.tests.ShooterCommandTest;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.MechanismViewer;
import frc.robot.subsystems.AmpHoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RedirectorsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.pivot.PivotIORobot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.roller.RollerMotorIORobot;
import frc.robot.subsystems.roller.RollerMotorIOSim;
import frc.robot.subsystems.shooter.TalonVelocityIORobot;
import frc.robot.subsystems.shooter.TalonVelocityIOSim;
import frc.robot.subsystems.timeofflight.TimeOfFlightIORobot;
import frc.robot.subsystems.timeofflight.TimeOfFlightIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretIORobot;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.LimeLightDetectionUtility;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.SpeakerScoreUtility;
import frc.robot.utilities.StartInTeleopUtility;
import frc.robot.utilities.LimelightHelpers.Results;
import frc.robot.utilities.SpeakerScoreUtility.Target;

import java.time.Instant;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final boolean USE_LIMELIGHT_APRIL_TAG = true;
    private boolean m_visionUpdatesOdometry = true;
    
    //The position we want the eleveator to move to.
    private final double ENDGAME_TARGET_POSITION = 0.0;
    private final double ENDGAME_DEFAULT_POSITION = 0.0;

    private static final double POV_PERCENT_SPEED = 1.0;
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1;
    private static final double PERCENT_SPEED = 1.0;

    private static final String CANBUS = "rio";

    //--DIO IDS--\\

    private static final int TURRET_ENCODER_DIO = 1;
    private static final double TURRET_OFFSET = 329.14;// 193.0; // -167.0 //if negative value, add 360

    private static final double TURRET_MANUAL_SPEED = 0.2;

    private static final double INTAKE_SUPPLY_CURRENT_LIMIT = 30.0;
    private static final double INTAKE_STATOR_CURRENT_LIMIT = 80.0;


    private LimeLightDetectionUtility m_LimeLightDetectionUtility = new LimeLightDetectionUtility("limelight-game");
    //#endregion

    //Use max speed from tuner constants from webpage
    static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    final double MaxAngularRate = 1.5 * 2.0 * Math.PI; // 1 rotation per second max angular velocity  

    /* Setting up bindings for necessary control of the swerve drive platform */
    SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private StartInTeleopUtility m_StartInTeleopUtility = new StartInTeleopUtility(drivetrain::seedFieldRelative);

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-cen

    //--PID AND FF CONSTANTS--\\

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
        .withKP(55.0)
        .withKI(0) 
        .withKD(0) 
        .withKA(0) 
        .withKG(0.35) // MotionMagic voltage
        .withKS(0) 
        .withKV(0);
    
    //SPEAKER SHOT
    private final Slot0Configs shooterLeftS0C = 
      new Slot0Configs()
        .withKP(42.0) //45.0 // 55 when 140 set  but issues with motor moving after going back to 0
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); //4.0

    // 0.08 on kP and 0.0 on kS if using voltage

    //SPEAKER SHOT
    private final Slot0Configs shooterRightS0C =
      new Slot0Configs()
        .withKP(39.6) //45.0 // 55 when 140 set  but issues with motor moving after going back to 0
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); //4.0

    // 0.08 on kP and 0.0 on kS if using voltage

    //FEED SHOT
    private final Slot1Configs shooterLeftS1C =
      new Slot1Configs()
        .withKP(10.0) //45.0 // 55 when 140 set  but issues with motor moving after going back to 0
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); //4.0

    //FEED SHOT
    private final Slot1Configs shooterRightS1C =
      new Slot1Configs()
        .withKP(10.0) //45.0 // 55 when 140 set  but issues with motor moving after going back to 0
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); //4.0

    private final Slot0Configs turretS0C =
      new Slot0Configs()
        .withKP(175.0)
        .withKI(0.0) 
        .withKD(0.0) 
        .withKA(0.0) 
        .withKG(0.0) // MotionMagic voltage
        .withKS(0.35) 
        .withKV(0.0);


    private final Slot0Configs pivotSimS0C =
      new Slot0Configs()
        .withKP(150.0)
        .withKI(0) 
        .withKD(0) 
        .withKA(0) 
        .withKG(0.35) // MotionMagic voltage
        .withKS(0) 
        .withKV(0);

        // 0.26 kp. Set to 0.1 for testing
    private final ProfiledPIDController turretPID = new ProfiledPIDController(
      // 0.1, 0.0, 0.0, new Constraints(0.1, 0.0) 
      0, 0, 0, new Constraints(0.0, 0.0) // zero'd values for safety, we dont want turret to move
    ); //TODO: Set good vals
    // ks overcomes friction on the turret
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
      // 0.375, 0.0, 0.0
      0, 0, 0 // zero'd values for safety, we dont want turret to move
    ); 

    
    //--MOTION MAGIC CONSTANTS--\\
    
    private final MotionMagicConfigs climbingMMC = 
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(5)
        .withMotionMagicExpo_kV(1)
        .withMotionMagicExpo_kA(4);

    private final MotionMagicConfigs pivotMMC =
      new MotionMagicConfigs() // Currently set slow
        .withMotionMagicAcceleration(3.0) //18.0 fast values (but slam at zero set point)
        .withMotionMagicCruiseVelocity(4.0)//11.0 fast values (but slam at zero set point)
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0);

    private final MotionMagicConfigs pivotSimMMC =
      new MotionMagicConfigs() // Currently set slow
        .withMotionMagicAcceleration(30.0) //18.0 fast values (but slam at zero set point)
        .withMotionMagicCruiseVelocity(400.0)//11.0 fast values (but slam at zero set point)
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0);

    private final MotionMagicConfigs shooterMMC =
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0)
        .withMotionMagicCruiseVelocity(0.0); //TODO set vals

    private final MotionMagicConfigs turretMMC =
      new MotionMagicConfigs() // Currently set slow
        .withMotionMagicAcceleration(10.0) 
        .withMotionMagicCruiseVelocity(0.5)
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0);
    
    public static final double TURRET_REFINE_COMMAND_VELOCITY = 0.1;
    public static final double TURRET_REFINE_COMMAND_ACCELERATION = 1.0;
    public static final double TURRET_REFINE_COMMAND_JERK = 0.0;
    public static final boolean TURRET_REFINE_COMMAND_ENABLEFOC = false;
    public static final double TURRET_REFINE_COMMAND_FEED_FORWARD = 0.0;
    public static final int TURRET_REFINE_COMMAND_SLOT = 0;
    public static final boolean TURRET_REFINE_COMMAND_OVERRIDE_BRAKE_DUR_NEUTRAL = false;
    public static final boolean TURRET_REFINE_COMMAND_LIMIT_FORWARD_MOTION = false;
    public static final boolean TURRET_REFINE_COMMAND_LIMIT_REVERSE_MOTION = false;

    public static final boolean ENDGAME_ELEVATOR_ENABLEFOC = false;
    public static final double ENDGAME_ELEVATOR_JERK = 0.0;
    public static final double ENDGAME_ELEVATOR_ACCELERATION = 1.0;
    public static final double ENDGAME_ELEVATOR_VELOCITY = 10.0;
    public static final boolean ENDGAME_ELEVATOR_OVERRIDEBRAKEDURNEUTRAL = false;
    public static final int ENDGAME_ELEVATOR_SLOT = 0;
    public static final double ENDGAME_ELEVATOR_FEEDFORWARD = 0.0;
    public static final boolean ENDGAME_ELEVATOR_LIMITFORWARDMOTION = false;
    public static final boolean ENDGAME_ELEVATOR_LIMITREVERSEMOTION = false;
    private static final boolean DELTA_LOGGING_ENABLED = false;

    //--VisionSTDsDevConstants--\\
    // TODO configure for april tag confidence level 
    //https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#setVisionMeasurementStdDev
    // Implement visionSTDsDevs into our code with default values
    private Matrix<N3, N1> visionSTDsDevs = VecBuilder.fill(0.5, 0.5, 0.5);
  
    //--SUBSYSTEMS--\\

    // public final ElevatorSubsystem m_climbingElevatorSubsystem = new ElevatorSubsystem(
    //   Robot.isReal()
    //     ? new ElevatorIORobot(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR)
    //     : new ElevatorIOSim(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR));

    private final RedirectorsSubsystem m_RedirectorsSubsystem = new RedirectorsSubsystem(4);

    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(
      Robot.isReal()
        ? new PivotIORobot(5, 22, CANBUS, 61.352413, pivotS0C, pivotMMC)
        : new PivotIOSim(5, 22, CANBUS, 15.0, pivotS0C, pivotMMC), //TODO ensure correct IDs
      Robot.isReal() 
        ? new TimeOfFlightIORobot(15, 62.0) 
        : new TimeOfFlightIOSim(15));

    private final TalonPosIO m_turretIO = Robot.isReal()
    ? new TurretIORobot(6,TURRET_ENCODER_DIO,CANBUS, 40, turretS0C, turretMMC,TURRET_OFFSET)
    : new TurretIOSim(6,0,CANBUS, 40, turretS0C, turretMMC,0.0);

    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem(m_turretIO);

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
        // 14,
        Robot.isReal() 
        ? new TalonVelocityIORobot(14, 0.666667, shooterLeftS0C, shooterLeftS1C, shooterMMC) 
        : new TalonVelocityIOSim(14, 0.666667 /* The gear ratio is 1.5:1. Therefore 1/1.5 */, shooterLeftS0C, shooterLeftS1C, shooterMMC),
        Robot.isReal() 
        ? new TalonVelocityIORobot(15, 0.666667, shooterRightS0C, shooterRightS1C, shooterMMC) 
        : new TalonVelocityIOSim(15, 0.666667 /* The gear ratio is 1.5:1. Therefore 1/1.5 */, shooterRightS0C, shooterRightS1C, shooterMMC));

    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(20, CANBUS) : new RollerMotorIOSim(20, CANBUS),
        Robot.isReal() ? new RollerMotorIORobot(3, CANBUS) : new RollerMotorIOSim(3, CANBUS),
        Robot.isReal() ? new TimeOfFlightIORobot(2, 200) : new TimeOfFlightIOSim(2));

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        // TunerConstants.kCANbusName means same canivore as drivetrain
        Robot.isReal() ? new RollerMotorIORobot(19, TunerConstants.kCANbusName) : new RollerMotorIOSim(19, TunerConstants.kCANbusName, INTAKE_STATOR_CURRENT_LIMIT,INTAKE_SUPPLY_CURRENT_LIMIT),
        Robot.isReal() ? new RollerMotorIORobot(7, TunerConstants.kCANbusName) : new RollerMotorIOSim(7, TunerConstants.kCANbusName, INTAKE_STATOR_CURRENT_LIMIT, INTAKE_SUPPLY_CURRENT_LIMIT),
        Robot.isReal() ? new TimeOfFlightIORobot(1, 200) : new TimeOfFlightIOSim(1),
        Robot.isReal() ? new TimeOfFlightIORobot(3, 200) : new TimeOfFlightIOSim(3));

    // private final AmpHoodSubsystem m_ampHoodSubsystem = new AmpHoodSubsystem(
    //   Robot.isReal() ? new RollerMotorIORobot(3, CANBUS) : new RollerMotorIOSim(3, CANBUS));

    private MechanismViewer m_mechViewer = new MechanismViewer(m_pivotSubsystem, m_turretSubsystem); 
    private SpeakerScoreUtility m_speakerUtil = new SpeakerScoreUtility(m_turretSubsystem);
    
    private SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private Telemetry logger = new Telemetry(MaxSpeed); 
    
    private AutoCommandManager m_autoManager = new AutoCommandManager(drivetrain, 
      m_LimeLightDetectionUtility, 
      m_turretSubsystem, 
      m_shooterSubsystem, 
      m_indexerSubsystem, 
      m_speakerUtil, 
      m_intakeSubsystem, 
      m_pivotSubsystem);

    SwerveRequest.Idle idle = new SwerveRequest.Idle();

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  // Only wish to configure subsystem once in DisableInit() -- delayed so give the devices time to startup 
  private boolean m_subsystemsConfigured = false;
  private boolean m_TeleopInitalized = false; // only want some things to initialze once
  private double m_last_RIOFPGA_timestamp = -1.0;

  private int visioncounter = 0;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureCoDriverBindingsForTesting();
    configureDriverBindings();
    portForwardCameras();
    // set our own visionMeasurementDeviations
    drivetrain.setVisionMeasurementStdDevs(visionSTDsDevs);
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
  private void configureDriverBindings() {
    //#region Default commands
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            // Code originally from team number 1091 to help deal with deadband on joystick for swerve drive (ty)
            drivetrain.applyRequest(
              joystickDriveWithDeadband(
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX)
            ));

    // m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, CommandFactoryUtility.INTAKE_REJECT_SPEED));

    // m_indexerSubsystem.setDefaultCommand(new IndexerCommand(m_indexerSubsystem, 0.0));
    
    m_turretSubsystem.setDefaultCommand(
      new ConditionalCommand(
        new TurretAimCommand(m_turretSubsystem), 
        new SetTurretPositionCommand(m_turretSubsystem, CommandFactoryUtility.TURRET_STOW_POS), 
        () -> m_indexerSubsystem.getSensor() && !m_turretSubsystem.getTurretLock()));
       
        // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    
    // Makes the shooter target presets
    m_driverController.povUp()
      .onTrue(m_speakerUtil.setDesiredTargetCommand(Target.far)); 
    m_driverController.povLeft().or(m_driverController.povRight())
      .onTrue(m_speakerUtil.setDesiredTargetCommand(Target.medium)); 
    m_driverController.povDown()
      .onTrue(m_speakerUtil.setDesiredTargetCommand(Target.close)); 
    //m_driverController.povUp().onTrue(new InstantCommand(() -> m_turretSubsystem.toggleTurretLock()));

    //Feed shot button
    m_driverController.x()
      .onTrue(CommandFactoryUtility.createFeedCommand(m_pivotSubsystem, m_shooterSubsystem, m_indexerSubsystem ))
      .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_turretSubsystem))
    ;
    
    // Eject shooter button
    m_driverController.y()
      .onTrue(CommandFactoryUtility.createEjectCommand(m_turretSubsystem, m_indexerSubsystem, m_intakeSubsystem))
      .onFalse(
        CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_turretSubsystem)
        .alongWith(m_intakeSubsystem.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED))
      )
    ;

    // m_driverController.a()
    // .onTrue(CommandFactoryUtility.createPrepareStarAmpCommand(m_indexerSubsystem, m_turretSubsystem, m_pivotSubsystem));
    

    //#region POV controls

    // m_driverController.pov(0).whileTrue(
    //   drivetrain.applyRequest(() -> forwardStraight.withVelocityX(POV_PERCENT_SPEED * MaxSpeed).withVelocityY(0.0)
    //   ));
    // m_driverController.pov(180).whileTrue(
    //   drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-POV_PERCENT_SPEED * MaxSpeed).withVelocityY(0.0)
    //   ));
    // m_driverController.pov(90).whileTrue(
    //   drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(-POV_PERCENT_SPEED * MaxSpeed)
    //   ));
    // m_driverController.pov(270).whileTrue(
    //   drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(POV_PERCENT_SPEED * MaxSpeed)
    //   ));
    //#endregion

    //#region Trigger/Bumper controls
    // reset the field-centric heading on left bumper press TODO test
    // m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    // Intake button
    m_driverController.leftBumper()
      .whileTrue(CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem, m_turretSubsystem))
        .onFalse(CommandFactoryUtility.createNoteBackUpCommand(m_indexerSubsystem, m_intakeSubsystem, false));
    ;

    // Game-Piece Intake
    m_driverController.leftTrigger()
      .whileTrue( new LimeLightIntakeCommand(drivetrain, m_LimeLightDetectionUtility, m_driverController)
        .alongWith(CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem, m_turretSubsystem)))
          .onFalse(CommandFactoryUtility.createNoteBackUpCommand(m_indexerSubsystem, m_intakeSubsystem, false));
    ;
    
    // Auto Aim Shoot
    m_driverController.rightBumper().whileTrue(
      new ConditionalCommand(
        new RepeatCommand(CommandFactoryUtility.createPivotAndShooterSpeedCommand(m_shooterSubsystem, m_pivotSubsystem, null)),
        new InstantCommand(),
        () -> m_speakerUtil.getAutoAim()
      )
    );

    // Amp Button
    m_driverController.rightTrigger()
      .onTrue(CommandFactoryUtility.createStarAmpCommand(m_indexerSubsystem, m_turretSubsystem, m_pivotSubsystem))
      .onFalse(CommandFactoryUtility.createStopStarAmpCommand(m_indexerSubsystem, m_turretSubsystem, m_pivotSubsystem))
    ;
     

    // Speaker score button
    m_driverController.rightBumper().whileTrue(
        new ConditionalCommand(
          new WaitCommand(0.2).until(() -> m_pivotSubsystem.getPosition()>0.0)
            .andThen(CommandFactoryUtility.createSpeakerScoreCommand(m_speakerUtil, m_shooterSubsystem, m_pivotSubsystem, m_indexerSubsystem, m_turretSubsystem, null, false)),
          new InstantCommand(
            () -> {
              double angle = m_speakerUtil.getPivotAngle();
              double lspeed = m_speakerUtil.getLeftShooterSpeed();
              double rspeed = m_speakerUtil.getRightShooterSpeed();
              m_shooterSubsystem.setSpeed(lspeed, rspeed);
              m_pivotSubsystem.setPosition(angle);
            }
          ).andThen(CommandFactoryUtility.createSpeakerScoreCommand(m_speakerUtil, m_shooterSubsystem, m_pivotSubsystem, m_indexerSubsystem, m_turretSubsystem, null, false))
          ,
          () -> m_speakerUtil.getAutoAim()
        )
    )
    .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_turretSubsystem));
    
    //#endregion 

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** 
   * <h3>joystickDriveWithDeadband</h3>
   * Applies deadband to joystick values and returns a request for drivetrain
   * 
   * <b>Code originally from team number 1091 to help deal with deadband on joystick for swerve drive</b>
   * @param xSupplier supplier for joystick's x axis
   * @param ySupplier supplier for joystick's y axis
   * @param omegaSupplier supplier for omega
   * @return request for drivetrain
   */
  private Supplier<SwerveRequest> joystickDriveWithDeadband(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return () -> {  
        double xValue = xSupplier.getAsDouble();   
        double yValue = ySupplier.getAsDouble();
        double omegaValue = omegaSupplier.getAsDouble();       
           
        Translation2d linearVelocity = getLinearVelocity(xValue, yValue);
                        
        // Squaring the omega value and applying a deadband 
        double omega = MathUtil.applyDeadband(-omegaValue, JOYSTICK_ROTATIONAL_DEADBAND);
        omega = Math.copySign(omega * omega, omega);


        return drive.withVelocityX(scaleLinearVelocity(linearVelocity.getX()))
          .withVelocityY(scaleLinearVelocity(linearVelocity.getY()))
          .withRotationalRate(omega * MaxAngularRate); // Drive counterclockwise with negative X (left)
      };
  }

  public static Translation2d getLinearVelocity(double xValue, double yValue) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(
                    Math.hypot(xValue, yValue), JOYSTICK_DEADBAND);
    Rotation2d linearDirection =
            new Rotation2d(-xValue, -yValue);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calculate new linear velocity
    Translation2d linearVelocity =
            new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();
    return linearVelocity;
  }

  public static double scaleLinearVelocity(double value) {
    return value*MaxSpeed*PERCENT_SPEED;
  }
  
  @Deprecated
  private void configureCoDriverBindingsForTesting() {
    //#region Test Commands

    m_coDriverController.b().onTrue(m_RedirectorsSubsystem.getNewExtendCommand())
    .onFalse(new InstantCommand(() -> m_RedirectorsSubsystem.setVoltage(0.0)));
    m_coDriverController.a().onTrue(m_RedirectorsSubsystem.getNewRetractCommand())
    .onFalse(new InstantCommand(() -> m_RedirectorsSubsystem.setVoltage(0.0)));
  
    // m_coDriverController.b().whileTrue(new SetTurretPositionCommandTest(m_turretSubsystem, 0));
    
    m_coDriverController.leftTrigger().whileTrue(new IntakeCommandTest(m_intakeSubsystem,0.0/100.0));
    m_coDriverController.rightTrigger().whileTrue(new ShooterCommandTest(m_shooterSubsystem,0.0,0.0,true)
    .alongWith(new IndexerCommandTest(m_indexerSubsystem, 0.0)))
    .onFalse(new IndexerCommand(m_indexerSubsystem, 0.0)
    .alongWith(new ShooterCommand(m_shooterSubsystem, 0.0, 0.0)));
    m_coDriverController.rightBumper().whileTrue(new ShooterCommand(m_shooterSubsystem, -0.8, -0.8).raceWith(new IndexerCommand(m_indexerSubsystem, 0.2)));
    m_coDriverController.x().whileTrue(new IndexerCommandTest(m_indexerSubsystem, 0.0));
    // m_coDriverController.b().whileTrue(new IndexerCommandTest(m_indexerSubsystem, 0.0).until(m_indexerSubsystem::getSensor));
    // m_coDriverController.a().whileTrue((new ShooterCommandTest(m_shooterSubsystem,0.0/100.0,0.0/100.0))
    //   .alongWith(new SetPivotPositionCommandTest(m_pivotSubsystem, 90)))
    //   .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_turretSubsystem));
    // m_coDriverController.y().whileTrue(new SetPivotPositionCommandTest(m_pivotSubsystem, 0.0));
    //#endregion

    m_coDriverController.rightBumper().whileTrue(new TurretRefineCommand(m_turretSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    if (autoCommand != null) {
        m_StartInTeleopUtility.updateAutonomous();
    }
    m_visionUpdatesOdometry = false; // Turns off vision updates for autonomous
    return autoCommand;
  }

  public void robotPeriodic() {
    double fpgaTimestampStart = Logger.getRealTimestamp();
    updateAllVision();
    fpgaTimestampStart=logTimestamp(fpgaTimestampStart, "updateAllVision", this);
    m_mechViewer.periodic();
    fpgaTimestampStart = logTimestamp(fpgaTimestampStart, "mechViewer", this);
    Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distance", SpeakerScoreUtility.inchesToSpeaker());
    Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/inverseTanPivotAngleUnimplemented", 
      SpeakerScoreUtility.computePivotAngleInverseTan(SpeakerScoreUtility.inchesToSpeaker()));
    fpgaTimestampStart=logTimestamp(fpgaTimestampStart, "speakerScoreCalc", this);  
  }
   /**Creates a method for logging the Delta Time in the console. Can be disabled in the constants. 
   * 
   * @param startTimestamp
   * @param loggerName
   * @return
   */
  public double logTimestamp(double startTimestamp, String loggerName, Object classObject){
    double timestamp = 0;
    if(DELTA_LOGGING_ENABLED){
      timestamp = Logger.getRealTimestamp();

      Logger.recordOutput(classObject.getClass().getSimpleName()+ "/"+loggerName,( startTimestamp-timestamp)/1000000.0);
    }
    return timestamp;
  }

  /*
   * Sets the port offsets
   */
  public void portForwardCameras() {
    PortForwarder.add(5800, "10.9.30.30", 5801); //limelight-front
    PortForwarder.add(5801, "10.9.30.31", 5801); //limelight-right
    PortForwarder.add(5802, "10.9.30.32", 5801); //limelight-left
    PortForwarder.add(5803, "10.9.30.33", 5801); //limelight-back
    PortForwarder.add(5804, "10.9.30.34", 5801); //limelight-game 
  }

  /**
   * Update all vision
   */
  public void updateAllVision() {
    if (USE_LIMELIGHT_APRIL_TAG) {  
      updatePoseEstimateWithAprilTags("limelight-front",true);
      updatePoseEstimateWithAprilTags("limelight-back",true);
      updatePoseEstimateWithAprilTags("limelight-right", true);
      updatePoseEstimateWithAprilTags("limelight-left", true);
    }
  }

  // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L57
  public void updatePoseEstimateWithAprilTags(String limeLightName, boolean usepose) {
    LimelightHelpers.PoseEstimate lastResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
    double fpgaTimestamp = Timer.getFPGATimestamp();
    // double fpgaTimestamp = Logger.getRealTimestamp();

    // distance from current pose to vision estimated pose
    Translation2d translation = drivetrain.getState().Pose.getTranslation();
    double poseDifference = translation.getDistance(lastResult.pose.getTranslation());

    double xyStds;
    double degStds;
    if (lastResult.tagCount >= 2) {
      xyStds = 0.1;
      degStds = 6;
    }
    // 1 target with large area and close to estimated pose
    else if (lastResult.tagCount == 1 && lastResult.avgTagArea > 0.8 && poseDifference < 0.5) {
      xyStds = 1.0;
      degStds = 12;
    }
    // conditions don't match to add a vision measurement
    else {
      SmartDashboard.putBoolean(limeLightName + "/Updated", false);
      return;
    }

    if (m_visionUpdatesOdometry&&usepose) {
        m_StartInTeleopUtility.updateTags();

        drivetrain.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

        drivetrain.addVisionMeasurement(lastResult.pose,  
          fpgaTimestamp - (lastResult.latency/1000.0));
        
        SmartDashboard.putBoolean(limeLightName + "/Updated", true);
        Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/UpdatCounts", this.visioncounter);
        this.visioncounter++;
    }
    
    Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", lastResult.pose);
  }
  
  public void simulationPeriodic() {
    // mechanismSimulator.periodic(); // Moved to robotPeriodic()
  }

  public void teleopInit() {
    SpeakerScoreUtility.resetShotSpeedOffset();
    SpeakerScoreUtility.resetShotOffset();// Makes sure auto offets do not continue thru teleop.
    if(!m_TeleopInitalized) {
      // Only want to initialize starting position once (if teleop multiple times dont reset pose again)
      m_StartInTeleopUtility.updateStartingPosition(); 
      m_TeleopInitalized = true;
      m_visionUpdatesOdometry = true;
    }
  }

  // Configures
  public void disabledInit() {
    // Only configure once
    if(!m_subsystemsConfigured) {
      m_turretIO.delayedConfigure();
      m_subsystemsConfigured = true;
    }
  }
}

