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
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorIORobot;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorType;
import frc.robot.subsystems.mm_turret.mmTurretIORobot;
import frc.robot.subsystems.mm_turret.mmTurretIOSim;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;
import frc.robot.subsystems.pivot.PivotIORobot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.roller.RollerMotorIORobot;
import frc.robot.subsystems.roller.RollerMotorIOSim;
import frc.robot.subsystems.shooter.TalonVelocityIORobot;
import frc.robot.subsystems.shooter.TalonVelocityIOSim;
import frc.robot.subsystems.timeofflight.TimeOfFlightIORobot;
import frc.robot.subsystems.timeofflight.TimeOfFlightIOSim;
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
    private static final double TURRET_OFFSET = 13.7;// 193.0; // -167.0 //TODO: if negative value, add 360

    private static final double TURRET_MANUAL_SPEED = 0.2;

    private static final double INTAKE_SUPPLY_CURRENT_LIMIT = 30.0;
    private static final double INTAKE_STATOR_CURRENT_LIMIT = 150.0;


    private LimeLightDetectionUtility m_LimeLightDetectionUtility = new LimeLightDetectionUtility("limelight-game");
    //#endregion

    //Use max speed from tuner constants from webpage
    static final double MaxSpeed = TunerConstants.kMaxSpeed;
    final double MaxAngularRate = 1.5 * 2.0 * Math.PI; // 1 rotation per second max angular velocity  

    /* Setting up bindings for necessary control of the swerve drive platform */
    SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
    private StartInTeleopUtility m_StartInTeleopUtility = new StartInTeleopUtility(drivetrain::seedFieldRelative);

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-cen

    //--PID AND FF CONSTANTS--\\
    private final Slot0Configs shootingElevatorS0C = 
      new Slot0Configs()
        .withKP(12.0)
        .withKI(0.0)
        .withKD(0)
        .withKA(0)
        .withKG(0.5)
        .withKS(0)
        .withKV(0);

    private final Slot0Configs shootingS0CSimulation = 
      new Slot0Configs()
        .withKP(1)
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
        .withKP(55.0)
        .withKI(0) 
        .withKD(0) 
        .withKA(0) 
        .withKG(0.35) // MotionMagic voltage
        .withKS(0) 
        .withKV(0);
    
    private final Slot0Configs shooterS0C =
      new Slot0Configs()
        .withKP(35.0) //45.0 // 55 when 140 set  but issues with motor moving after going back to 0
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); //4.0

    // 0.08 on kP and 0.0 on kS if using voltage

    private final Slot0Configs turretS0C =
      new Slot0Configs()
        .withKP(175.0)
        .withKI(0.0) 
        .withKD(0.0) 
        .withKA(0.0) 
        .withKG(0.0) // MotionMagic voltage
        .withKS(0.35) 
        .withKV(0.0);

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
    
    private final MotionMagicConfigs shootingElevatorMMC = 
    // We used motion magic voltage when tuning
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(1.0)
        .withMotionMagicCruiseVelocity(10.0)
        .withMotionMagicExpo_kV(1.0)
        .withMotionMagicExpo_kA(4.0);
    
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

    private final MotionMagicConfigs shooterMMC =
      new MotionMagicConfigs()
        .withMotionMagicAcceleration(0)
        .withMotionMagicJerk(0); //TODO set vals

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

    //--VisionSTDsDevConstants--\\
    // TODO configure for april tag confidence level 
    //https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#setVisionMeasurementStdDev
    // Implement visionSTDsDevs into our code with default values
    private Matrix<N3, N1> visionSTDsDevs = VecBuilder.fill(0.5, 0.5, 0.5);
  
    //--SUBSYSTEMS--\\

    public final ElevatorSubsystem m_shootingElevatorSubsystem = new ElevatorSubsystem(
      Robot.isReal()
        ? new ElevatorIORobot(3, 4, CANBUS, shootingElevatorS0C, shootingElevatorMMC, ElevatorType.SHOOTING_ELEVATOR)
        : new ElevatorIOSim(3, 4, CANBUS, shootingS0CSimulation, shootingElevatorMMC, ElevatorType.SHOOTING_ELEVATOR));

    // public final ElevatorSubsystem m_climbingElevatorSubsystem = new ElevatorSubsystem(
    //   Robot.isReal()
    //     ? new ElevatorIORobot(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR)
    //     : new ElevatorIOSim(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR));

    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(
      Robot.isReal()
        ? new PivotIORobot(5, CANBUS, 61.352413, pivotS0C, pivotMMC)
        : new PivotIOSim(5, CANBUS, 61.352413, pivotS0C, pivotMMC), 
      Robot.isReal() 
        ? new TimeOfFlightIORobot(15, 62.0) 
        : new TimeOfFlightIOSim(15));

    private final TalonPosIO m_turretIO = Robot.isReal()
    ? new mmTurretIORobot(6,TURRET_ENCODER_DIO,CANBUS, 40, turretS0C, turretMMC,TURRET_OFFSET)
    : new mmTurretIOSim(6,0,CANBUS, 40, turretS0C, turretMMC,0.0);

    private final mmTurretSubsystem m_turretSubsystem = new mmTurretSubsystem(m_turretIO);

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
        // 14,
        Robot.isReal() ? new TalonVelocityIORobot(14, 0.5, shooterS0C, shooterMMC) : new TalonVelocityIOSim(14, 0.5, shooterS0C, shooterMMC) ,
        Robot.isReal() ? new TalonVelocityIORobot(15, 0.5, shooterS0C, shooterMMC)  : new TalonVelocityIOSim(15, 0.5, shooterS0C, shooterMMC));

    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(20, CANBUS) : new RollerMotorIOSim(20, CANBUS),
        Robot.isReal() ? new TimeOfFlightIORobot(2, 200) : new TimeOfFlightIOSim(2));


    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        // TunerConstants.kCANbusName means same canivore as drivetrain
        Robot.isReal() ? new RollerMotorIORobot(19, TunerConstants.kCANbusName) : new RollerMotorIOSim(19, TunerConstants.kCANbusName, INTAKE_STATOR_CURRENT_LIMIT,INTAKE_SUPPLY_CURRENT_LIMIT),
        Robot.isReal() ? new RollerMotorIORobot(7, TunerConstants.kCANbusName) : new RollerMotorIOSim(7, TunerConstants.kCANbusName, INTAKE_STATOR_CURRENT_LIMIT, INTAKE_SUPPLY_CURRENT_LIMIT),
        Robot.isReal() ? new TimeOfFlightIORobot(1, 200) : new TimeOfFlightIOSim(1),
        Robot.isReal() ? new TimeOfFlightIORobot(3, 200) : new TimeOfFlightIOSim(3));

    private MechanismViewer m_mechViewer = new MechanismViewer(m_pivotSubsystem, m_shootingElevatorSubsystem, m_shootingElevatorSubsystem, m_turretSubsystem); // TODO: 2 Shooting elevators given

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
      m_shootingElevatorSubsystem, 
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

    // Sets the desired positions for the speaker
    m_driverController.y().onTrue(m_speakerUtil.setDesiredTargetCommand(Target.far)); 
    m_driverController.x().or(m_driverController.b()).onTrue(m_speakerUtil.setDesiredTargetCommand(Target.medium)); 
    m_driverController.a().onTrue(m_speakerUtil.setDesiredTargetCommand(Target.close)); 

    // m_driverController.back().whileTrue(CommandFactoryUtility.createElevatorClimbCommand(m_shootingElevatorSubsystem))
    //   .onFalse(CommandFactoryUtility.createStowElevatorCommand(m_shootingElevatorSubsystem));
    
    //#region POV controls

    m_driverController.povUp().onTrue(new InstantCommand(() -> m_turretSubsystem.toggleTurretLock()));

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

    // Eject shooter button
    m_driverController.leftTrigger().whileTrue(CommandFactoryUtility.createEjectCommand(m_turretSubsystem, m_indexerSubsystem, m_intakeSubsystem))
      .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_shootingElevatorSubsystem, m_turretSubsystem)
            .alongWith(m_intakeSubsystem.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED)));
    
    // Intake button TODO Test
    m_driverController.leftBumper()
      .whileTrue(CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem, m_turretSubsystem))
      .onFalse(CommandFactoryUtility.createStopIntakingCommand(m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem)
        .andThen(m_intakeSubsystem.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED)))
      ;
    
    m_driverController.start().whileTrue(m_shootingElevatorSubsystem.newSetPosCommand(ENDGAME_TARGET_POSITION))
      .onFalse(m_shootingElevatorSubsystem.newPullCommand(ENDGAME_DEFAULT_POSITION))
      ;
      
    m_driverController.rightBumper().and(m_driverController.rightTrigger().negate()).whileTrue(
      new ConditionalCommand(
        new RepeatCommand(CommandFactoryUtility.createPivotAndShooterSpeedCommand(m_shooterSubsystem, m_pivotSubsystem, null)),
        new InstantCommand(),
        () -> m_speakerUtil.getAutoAim()
      )
    );
     

    // Speaker score button TODO: TEST CHANGES
    m_driverController.rightBumper().and(m_driverController.rightTrigger().negate()).whileTrue(
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
    .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_shootingElevatorSubsystem, m_turretSubsystem));

    m_driverController.rightTrigger()
    .whileTrue(
      new LimeLightIntakeCommand(drivetrain, m_LimeLightDetectionUtility, m_driverController::getLeftY)
      .alongWith(CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem, m_turretSubsystem)))
    .onFalse(
      CommandFactoryUtility.createStopIntakingCommand(m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem)
      .andThen(m_intakeSubsystem.newSetSpeedCommand(CommandFactoryUtility.INTAKE_REJECT_SPEED)))
    ;
    // Amp score button
    // m_driverController.rightBumper().and(m_driverController.rightTrigger())
    //   .whileTrue(CommandFactoryUtility.createAmpScoreCommand(m_shootingElevatorSubsystem, m_pivotSubsystem, m_shooterSubsystem, m_indexerSubsystem))
    //   .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_shootingElevatorSubsystem));
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

    m_coDriverController.b().whileTrue(new SetTurretPositionCommandTest(m_turretSubsystem, 0));
    
    m_coDriverController.leftTrigger().whileTrue(new IntakeCommandTest(m_intakeSubsystem,0.0/100.0));
    m_coDriverController.rightTrigger().whileTrue(new ShooterCommandTest(m_shooterSubsystem,0.0/100.0,0.0/100.0));
    m_coDriverController.rightBumper().whileTrue(new ShooterCommand(m_shooterSubsystem, -0.8, -0.8).raceWith(new IndexerCommand(m_indexerSubsystem, 0.2)));
    m_coDriverController.x().whileTrue(new IndexerCommandTest(m_indexerSubsystem, 0.0));
    // m_coDriverController.b().whileTrue(new IndexerCommandTest(m_indexerSubsystem, 0.0).until(m_indexerSubsystem::getSensor));
    m_coDriverController.a().whileTrue((new ShooterCommandTest(m_shooterSubsystem,0.0/100.0,0.0/100.0))
      .alongWith(new SetPivotPositionCommandTest(m_pivotSubsystem, 90)))
      .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem, m_pivotSubsystem, m_shootingElevatorSubsystem, m_turretSubsystem));
    m_coDriverController.y().whileTrue(new InstantCommand(()->m_pivotSubsystem.setPosition(0.0)));
    m_coDriverController.leftBumper().whileTrue(new SetElevatorPositionCommandTest(m_shootingElevatorSubsystem, 0));
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
    m_visionUpdatesOdometry = true; // Turns off vision updates for autonomous
    return autoCommand;
  }

  public void robotPeriodic() {
    updateAllVision();
    m_mechViewer.periodic();
    Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distance", SpeakerScoreUtility.inchesToSpeaker());
    Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/inverseTanPivotAngleUnimplemented", 
      SpeakerScoreUtility.computePivotAngleInverseTan(SpeakerScoreUtility.inchesToSpeaker()));
  }

  /*
   * Sets the port offsets
   */
  public void portForwardCameras() {
    PortForwarder.add(5800, "10.9.30.30", 5801); //limelight-front
    PortForwarder.add(5801, "10.9.30.31", 5801); //limelight-back
    PortForwarder.add(5802, "10.9.30.32", 5801); //limelight-game
    PortForwarder.add(5803, "10.9.30.33", 5801); //limelight-turret
  }

  /**
   * Update all vision
   */
  public void updateAllVision() {
    if (USE_LIMELIGHT_APRIL_TAG) {  
      // updateVisionOdometry("limelight-front");
      // updateVisionOdometry("limelight-back");
      updatePoseEstimatorWithVisionBotPose("limelight-front");
      updatePoseEstimatorWithVisionBotPose("limelight-back");
    }
  }

  public void updatePoseEstimatorWithVisionBotPose(String limeLightName) {
    Results lastResult = LimelightHelpers.getLatestResults(limeLightName).targetingResults;
    // invalid LL data
    if (lastResult.getBotPose2d_wpiBlue().getX() == 0.0) {
      SmartDashboard.putBoolean(limeLightName + "/Updated", false);
      return;
    }
    double fpgaTimestamp = Timer.getFPGATimestamp();

    // distance from current pose to vision estimated pose
    Translation2d translation = drivetrain.getState().Pose.getTranslation();
    double poseDifference = translation.getDistance(lastResult.getBotPose2d_wpiBlue().getTranslation());

    if (lastResult.valid) {
      double xyStds;
      double degStds;
      // multiple targets detected
      if (lastResult.targets_Fiducials.length >= 2) {
        xyStds = 0.1;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (LimelightHelpers.getTA(limeLightName) > 0.8 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (LimelightHelpers.getTA(limeLightName) > 0.1 && poseDifference < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      }
      // conditions don't match to add a vision measurement
      else {
        SmartDashboard.putBoolean(limeLightName + "/Updated", false);
        return;
      }

      this.visioncounter++;

      Logger.recordOutput("LimeLightOdometry/"+ limeLightName + "/UpdatCounts", this.visioncounter);

        m_StartInTeleopUtility.updateTags();
        int[] idArray = createAprilTagIDArray(lastResult); //Creates a local array to store all of the IDs that the Limelight saw
        Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/IDs", Arrays.toString(idArray));
        Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", lastResult.getBotPose2d_wpiBlue());

        if (m_visionUpdatesOdometry) {
            drivetrain.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
              drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiBlue(),  
            fpgaTimestamp - (lastResult.latency_pipeline/1000.0) //
              - (lastResult.latency_capture/1000.0) //
              - lastResult.latency_jsonParse / 1000.0 /*already in millis*/); // Due to json parsing in getlatestresults
            SmartDashboard.putBoolean(limeLightName + "/Updated", true);
        }
    }
  }
  

  /**
   * Prints limelight, and limelight name. If the last result was valid, and the length is bigger than 0.
   * If there is a alliance to get the alliance, and if its red it sets the alliance to red; otherwise it sets the alliance to blue.
   * @param limeLightName
   */
  public void updateVisionOdometry(String limeLightName) {
      Results lastResult = LimelightHelpers.getLatestResults(limeLightName).targetingResults;
      double fpgaTimestamp = Timer.getFPGATimestamp();
      boolean resultIsGood = false;

      if (isValidResult(lastResult)) { //Verifies that the Tags are valid, have values, and are between IDs 1 and 16
        // if(lastResult.timestamp_RIOFPGA_capture > m_last_RIOFPGA_timestamp) {
          int[] idArray = createAprilTagIDArray(lastResult); //Creates a local array to store all of the IDs that the Limelight saw

          //We use the percentage of the screen (TA) as a reference to distance
          if (idArray.length > 1) { //If the Limelight sees more than one Tag
              double area = 100.0; //Defaults to not using Tags
              /**
               * We sort by distances based on the groups of Tags because the Source Tags are farther apart than the Speaker Tags.
               * This makes the TA value bigger at the same distance away from the Tags.
              */
              if ((arrayContainsPair(idArray, 1, 2)) || (arrayContainsPair(idArray, 9, 10))) { //If the Limelight sees Source Tags
                  area = 0.6;//Source
              } else if ((arrayContainsPair(idArray, 3, 4)) || (arrayContainsPair(idArray, 7, 8))) { //If the Limelight sees Speaker Tags
                  area = 0.35; //Speaker
              } else { //If the Limelight sees more than two tags
                  area = 0.0; //Any
              }
                        
              if (LimelightHelpers.getTA(limeLightName) > area) {
                m_StartInTeleopUtility.updateTags();
                
                Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/IDs", Arrays.toString(idArray));
                Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", lastResult.getBotPose2d_wpiBlue());

                if (m_visionUpdatesOdometry) {
                    drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiBlue(), 
                    fpgaTimestamp - (lastResult.latency_pipeline/1000.0) //
                      - (lastResult.latency_capture/1000.0) //
                      - lastResult.latency_jsonParse / 1000.0/*already in millis*/); // Due to json parsing in getlatestresults
                    resultIsGood = true;
                }
              }
          }
          // m_last_RIOFPGA_timestamp = lastResult.timestamp_RIOFPGA_capture;
        // }
      }
       SmartDashboard.putBoolean(limeLightName + "/Updated", resultIsGood);
  }

  /**
   * 
   * Validates that the results the Limelight got are what we want to use
   * 
   * @param result The raw JSON dump from the Limelight
   * @return Boolean that says if we want to use our results
   */
  private boolean isValidResult(Results result) {
      return result.valid && result.targets_Fiducials.length > 0 &&
              result.targets_Fiducials[0].fiducialID >= 1 && result.targets_Fiducials[0].fiducialID <= 16;
  }

  /**
   * 
   * Returns a new array that contains all of the IDs that the Limelight sees
   * 
   * @param result The raw JSON dump from the Limelight
   * @return An array that contains all of the IDs that the Limelight sees
   */
  private int[] createAprilTagIDArray(Results result) {
      int[] idArray = new int[result.targets_Fiducials.length];
      for (int i = 0; i < result.targets_Fiducials.length; i++) {
          idArray[i] = (int) result.targets_Fiducials[i].fiducialID;
      }

      return idArray;
  }

  /**
   * 
   * Determines if both of the IDs are in the array
   * 
   * @param array An Array that contains all of the April Tag IDs
   * @param value1 ID number 1
   * @param value2 ID number 2
   * @return Boolean that returns if the two numbers are in the array
   */
  private boolean arrayContainsPair(int[] array, int value1, int value2) {
      return arrayContains(array, value1) && arrayContains(array, value2);
  }

  /**
   * 
   * Determines if the array contains a specific number
   * 
   * @param array An Array that contains all of the April Tag IDs
   * @param value ID value that is being checked
   * @return boolean that says if the number is in the array
   */
  private boolean arrayContains(int[] array, int value) {
      for (int i : array) {
          if (i == value) {
              return true;
          }
      }

      return false;
  }

  public void simulationPeriodic() {
    // mechanismSimulator.periodic(); // Moved to robotPeriodic()
  }

  public void teleopInit() {
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

