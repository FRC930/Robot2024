// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LimeLightIntakeCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.commands.SetPivotPositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurretAutoAimCommand;
import frc.robot.commands.TurretLimeLightAimCommand;
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
import frc.robot.subsystems.pivot.PivotIORobot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.roller.RollerMotorIORobot;
import frc.robot.subsystems.roller.RollerMotorIOSim;
import frc.robot.subsystems.shooter.TalonVelocityIORobot;
import frc.robot.subsystems.shooter.TalonVelocityIOSim;
import frc.robot.subsystems.timeofflight.TimeOfFlightIORobot;
import frc.robot.subsystems.timeofflight.TimeOfFlightIOSim;
import frc.robot.subsystems.turret.TurretIORobot;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.LimeLightDetectionUtility;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.SpeakerScoreUtility;
import frc.robot.utilities.StartInTeleopUtility;
import frc.robot.utilities.LimelightHelpers.Results;
import frc.robot.utilities.SpeakerScoreUtility.Target;

import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

    private final boolean UseLimeLightAprilTag = false; 

    private static final double POV_PERCENT_SPEED = 1.0;
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1;
    private static final double PERCENT_SPEED = 1.0;

    private static final String CANBUS = "rio";

    //--DIO IDS--\\

    private static final int TURRET_ENCODER_DIO = 0;
    private static final double TURRET_OFFSET = 281.87;


    private LimeLightDetectionUtility m_LimeLightDetectionUtility = new LimeLightDetectionUtility("limelight-game");
    //#endregion

    //Use max speed from tuner constants from webpage
    final double MaxSpeed = TunerConstants.kMaxSpeed;
    final double MaxAngularRate = 1.5 * Math.PI; // 3/4 a rotation per second max angular velocity  

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
        .withKP(36.0)
        .withKI(0) 
        .withKD(0) 
        .withKA(0) 
        .withKG(0.45) // MotionMagic voltage
        .withKS(0) 
        .withKV(0);
    
    private final Slot0Configs shooterS0C =
      new Slot0Configs()
        .withKP(30.0) 
        .withKI(0) 
        .withKD(0) 
        .withKG(0)
        .withKS(4.0); 

    private final ProfiledPIDController turretPID = new ProfiledPIDController(0.26, 0.0, 0.0, new Constraints(0.0, 0.0)); //TODO: Set good vals
    // ks overcomes friction on the turret
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(0.375, 0.0, 0.0); 

    
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
    
    //--VisionSTDsDevConstants--\\
    // TODO configure for april tag confidence level 
    //https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/mechanisms/swerve/SwerveDrivetrain.html#setVisionMeasurementStdDev
    // Implement visionSTDsDevs into our code with default values
    private Matrix<N3, N1> visionSTDsDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  
    //--SUBSYSTEMS--\\

    public final ElevatorSubsystem m_shootingElevatorSubsystem = new ElevatorSubsystem(
      Robot.isReal()
        ? new ElevatorIORobot(3, 4, CANBUS, shootingElevatorS0C, shootingElevatorMMC, ElevatorType.SHOOTING_ELEVATOR)
        : new ElevatorIOSim(3, 4, CANBUS, shootingS0CSimulation, shootingElevatorMMC, ElevatorType.SHOOTING_ELEVATOR));

    public final ElevatorSubsystem m_climbingElevatorSubsystem = new ElevatorSubsystem(
      Robot.isReal()
        ? new ElevatorIORobot(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR)
        : new ElevatorIOSim(21, 22, CANBUS, climbingS0C,  climbingMMC, ElevatorType.CLIMBING_ELEVATOR));

    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(
      Robot.isReal()
        ? new PivotIORobot(5, CANBUS, 61.352413, pivotS0C, pivotMMC)
        : new PivotIOSim(5, CANBUS, 61.352413, pivotS0C, pivotMMC));

    // TODO: Figure out real motor and encoder id
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem(
      Robot.isReal()
        ? new TurretIORobot(6, TURRET_ENCODER_DIO, CANBUS, 40, TURRET_OFFSET)
        : new TurretIOSim(6, TURRET_ENCODER_DIO, CANBUS, 40, TURRET_OFFSET), 
        turretPID, turretFF);

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
        Robot.isReal() ? new TalonVelocityIORobot(14, 1, shooterS0C, shooterMMC) : new TalonVelocityIOSim(14, 1, shooterS0C, shooterMMC) ,
        Robot.isReal() ? new TalonVelocityIORobot(15, 1, shooterS0C, shooterMMC)  : new TalonVelocityIOSim(15, 1, shooterS0C, shooterMMC));

    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(
        Robot.isReal() ? new RollerMotorIORobot(20, CANBUS) : new RollerMotorIOSim(20, CANBUS),
        Robot.isReal() ? new TimeOfFlightIORobot(2, 200) : new TimeOfFlightIOSim(2));


    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
        // TunerConstants.kCANbusName means same canivore as drivetrain
        Robot.isReal() ? new RollerMotorIORobot(19, TunerConstants.kCANbusName) : new RollerMotorIOSim(19, TunerConstants.kCANbusName),
        Robot.isReal() ? new RollerMotorIORobot(7, TunerConstants.kCANbusName) : new RollerMotorIOSim(7, TunerConstants.kCANbusName),
        Robot.isReal() ? new TimeOfFlightIORobot(1, 200) : new TimeOfFlightIOSim(1),
        Robot.isReal() ? new TimeOfFlightIORobot(3, 200) : new TimeOfFlightIOSim(3));

    MechanismViewer m_mechViewer = new MechanismViewer(m_pivotSubsystem, m_shootingElevatorSubsystem, m_climbingElevatorSubsystem, m_turretSubsystem);

    SpeakerScoreUtility m_speakerUtil = new SpeakerScoreUtility();
    
    SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(MaxSpeed);
    
    private AutoCommandManager m_autoManager = new AutoCommandManager(drivetrain, m_LimeLightDetectionUtility);

    SwerveRequest.Idle idle = new SwerveRequest.Idle();

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureCoDriverBindingsForTesting();
    configureDriverBindings();
    configureNamedCommands();
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
            drivetrain.applyRequest( // Code originally from team number 1091 to help deal with deadband on joystick for swerve drive
                            () -> {  
                                DoubleSupplier xSupplier = m_driverController::getLeftY;   
                                DoubleSupplier ySupplier = m_driverController::getLeftX;
                                DoubleSupplier omegaSupplier = m_driverController::getRightX;       
                                   
                                // Apply deadband
                                double linearMagnitude = MathUtil.applyDeadband(
                                                Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), JOYSTICK_DEADBAND);
                                Rotation2d linearDirection =
                                        new Rotation2d(-xSupplier.getAsDouble(), -ySupplier.getAsDouble());

                                // Square values
                                linearMagnitude = linearMagnitude * linearMagnitude;

                                // Calculate new linear velocity
                                Translation2d linearVelocity =
                                        new Pose2d(new Translation2d(), linearDirection)
                                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                                .getTranslation();
                                                
                                // Squaring the omega value and applying a deadband 
                                double omega = MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), JOYSTICK_ROTATIONAL_DEADBAND);
                                omega = Math.copySign(omega * omega, omega);


                                  return drive.withVelocityX(linearVelocity.getX() * MaxSpeed * PERCENT_SPEED)
                                    .withVelocityY(linearVelocity.getY() * MaxSpeed * PERCENT_SPEED)
                                    .withRotationalRate(omega * MaxAngularRate); // Drive counterclockwise with negative X (left)
                              }
              ));

    m_intakeSubsystem.setDefaultCommand(new IntakeCommand(m_intakeSubsystem, CommandFactoryUtility.INTAKE_REJECT_SPEED));
    
    m_turretSubsystem.setDefaultCommand(
      new ConditionalCommand(
        new TurretAutoAimCommand(m_turretSubsystem), 
        new SetTurretPositionCommand(m_turretSubsystem, CommandFactoryUtility.TURRET_STOW_POS), 
        m_indexerSubsystem::getSensor));
          
    // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // //TODO Test
    // //AMP position button
    // m_driverController.y()
    //   .whileTrue(new SetPivotPositionCommand(m_pivotSubsystem, PIVOT_AMP_POS)
    //     .alongWith(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, ELEVATOR_AMP_POS)
    //     .alongWith(new SetTurretPositionCommand(m_turretSubsystem, TURRET_STOW_POS))))
    //   .onFalse(new SetPivotPositionCommand(m_pivotSubsystem, PIVOT_STOW_POS)
    //     .alongWith(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, ELEVATOR_STOW_POS)));
    // // //TODO Test
    // m_driverController.b()
    //   .whileTrue(new SetPivotPositionCommand(m_pivotSubsystem, PIVOT_INTAKE_POS)
    //     .alongWith(new SetTurretPositionCommand(m_turretSubsystem, TURRET_STOW_POS)))
    //   .onFalse(new SetPivotPositionCommand(m_pivotSubsystem, PIVOT_STOW_POS)
    //     .alongWith(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, ELEVATOR_STOW_POS)));
    // m_driverController.x()
    //   .whileTrue(
    //     new ShooterCommand(m_shooterSubsystem,LEFT_SHOOTER_SPEAKER_SPEED, RIGHT_SHOOTER_SPEAKER_SPEED)
    //     .raceWith(new WaitCommand(1.0))
    //     .andThen(
    //       new IndexerCommand(m_indexerSubsystem, INDEXER_SPEAKER_SPEED)
    //       .until(()->!m_indexerSubsystem.getSensor() || m_driverController.getHID().getXButtonReleased())
    //     )
    //   ); //TODO review values and code

    m_driverController.y().onTrue(m_speakerUtil.setDesiredTargetCommand(Target.far));
    m_driverController.x().or(m_driverController.b()).onTrue(m_speakerUtil.setDesiredTargetCommand(Target.medium));
    m_driverController.a().onTrue(m_speakerUtil.setDesiredTargetCommand(Target.close));

    m_driverController.back().whileTrue(CommandFactoryUtility.createElevatorClimbCommand(m_shootingElevatorSubsystem))
      .onFalse(CommandFactoryUtility.createElevatorStowCommand(m_shootingElevatorSubsystem));
    
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

    // Eject shooter button
    m_driverController.leftTrigger().whileTrue(CommandFactoryUtility.createEjectCommand(m_shooterSubsystem, m_indexerSubsystem))
      .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem));
    
    // Intake button TODO Test
    m_driverController.leftBumper().whileTrue(CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem)); 

    // Speaker score button TODO: TEST CHANGES
    m_driverController.rightBumper().and(m_driverController.rightTrigger().negate()).whileTrue(
      new ConditionalCommand(
        new ShooterCommand(m_shooterSubsystem,LEFT_SHOOTER_SPEAKER_SPEEDS[0], RIGHT_SHOOTER_SPEAKER_SPEEDS[0])
          .alongWith(m_pivotSubsystem.newSetPosCommand(PIVOT_PIVOT_POSITIONS[0])),
        new ConditionalCommand(
          new ShooterCommand(m_shooterSubsystem,LEFT_SHOOTER_SPEAKER_SPEEDS[1], RIGHT_SHOOTER_SPEAKER_SPEEDS[1])
            .alongWith(m_pivotSubsystem.newSetPosCommand(PIVOT_PIVOT_POSITIONS[1])),
          new ShooterCommand(m_shooterSubsystem,LEFT_SHOOTER_SPEAKER_SPEEDS[2], RIGHT_SHOOTER_SPEAKER_SPEEDS[2])
            .alongWith(m_pivotSubsystem.newSetPosCommand(PIVOT_PIVOT_POSITIONS[2])),
          () -> m_speakerUtil.isMedium()
        ),
        () -> m_speakerUtil.isFar()
      )
      .alongWith(new WaitCommand(1.0).andThen(new IndexerCommand(m_indexerSubsystem, INDEXER_SPEAKER_SPEED)))
    )
    .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem));

    // Amp score button
    m_driverController.rightBumper().and(m_driverController.rightTrigger())
      .whileTrue(CommandFactoryUtility.createAmpShootCommand(m_shooterSubsystem, m_indexerSubsystem))
      .onFalse(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem));
    //#endregion 

    drivetrain.registerTelemetry(logger::telemeterize);
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
    m_coDriverController.a().whileTrue(new SetPivotPositionCommandTest(m_pivotSubsystem, 90));
    m_coDriverController.y().whileTrue(new InstantCommand(()->m_pivotSubsystem.setPosition(0.0)));
    m_coDriverController.leftBumper().whileTrue(new SetElevatorPositionCommandTest(m_shootingElevatorSubsystem, 0));
    //#endregion

    m_coDriverController.rightBumper().whileTrue(new TurretLimeLightAimCommand(m_turretSubsystem));

  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("aimAndShoot", 
        new TurretLimeLightAimCommand(m_turretSubsystem)
        .andThen(
          new ShooterCommand(m_shooterSubsystem,LEFT_SHOOTER_SPEAKER_SPEEDS[0], RIGHT_SHOOTER_SPEAKER_SPEEDS[0])
          .raceWith(new WaitCommand(1.0))
          .andThen(
            new IndexerCommand(m_indexerSubsystem, INDEXER_SPEAKER_SPEED))
          .andThen(new WaitCommand(0.25))
          .andThen(m_indexerSubsystem.newSetSpeedCommand(0.0)
            .alongWith(m_shooterSubsystem.newSetSpeedsCommand(0.0, 0.0)))
        ));
    NamedCommands.registerCommand("intake", CommandFactoryUtility.createRunIntakeCommand(m_intakeSubsystem, m_indexerSubsystem));
    NamedCommands.registerCommand("ampPosition", new SetPivotPositionCommand(m_pivotSubsystem, PIVOT_AMP_POS)
        .alongWith(new SetElevatorPositionCommand(m_shootingElevatorSubsystem, ELEVATOR_AMP_POS)
        .alongWith(new SetTurretPositionCommand(m_turretSubsystem, TURRET_STOW_POS))));
    NamedCommands.registerCommand("ampShoot", 
      CommandFactoryUtility.createAmpShootCommand(m_shooterSubsystem, m_indexerSubsystem)
        .andThen(new WaitCommand(0.5).andThen(CommandFactoryUtility.createStopShootingCommand(m_shooterSubsystem, m_indexerSubsystem)))
    );
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
    return autoCommand;
  }

  public void robotPeriodic() {
    updateAllVision();
    m_mechViewer.periodic();
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
    if (UseLimeLightAprilTag) {  
      updateVisionOdometry("limelight-front");
      updateVisionOdometry("limelight-back");
    }
  }

  /**
   * Prints limelight, and limelight name. If the last result was valid, and the length is bigger than 0.
   * If there is a alliance to get the alliance, and if its red it sets the alliance to red; otherwise it sets the alliance to blue.
   * @param limeLightName
   */
  public void updateVisionOdometry(String limeLightName) {
      boolean useResult = true;
      Results lastResult = LimelightHelpers.getLatestResults(limeLightName).targetingResults;
      if (lastResult.valid && lastResult.targets_Fiducials.length > 0 && lastResult.targets_Fiducials[0].fiducialID != 0) {
          if (lastResult.targets_Fiducials.length == 1) {
              if (LimelightHelpers.getTA(limeLightName) > 0.27) { //The robot must be close to use only one April Tag at a time
                useResult = true;
              } else {
                useResult = false;
              }
          } else {
              useResult = true;
          }

          if (useResult) { //Always update odometry through blue alliance because blue origin is always (0,0)
              m_StartInTeleopUtility.updateTags();
              drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp()); 
          }
      }
  }


  public void simulationPeriodic() {
    // mechanismSimulator.periodic(); // Moved to robotPeriodic()
  }

  public void teleopInit() {
    m_StartInTeleopUtility.updateStartingPosition();
  }
}

