// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.Results;
import frc.robot.commands.Autos;
import frc.robot.commands.SparkTestShooterCommand;
import frc.robot.commands.TestShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkMaxShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private static final double POV_PERCENT_SPEED = 0.3;
    private static final double JOYSTICK_DEADBAND = 0.1;
    private static final double JOYSTICK_ROTATIONAL_DEADBAND = 0.1;
    private static final double PERCENT_SPEED = 0.3;
    // MK3 Falcon 13.6 ft/s 8.16:1 or 16.2 ft/s 6.86:1
    // https://www.swervedrivespecialties.com/products/mk3-swerve-module?variant=31575980703857
    final double MaxSpeed = Units.feetToMeters(16.2); //13.6); //  meters per second desired top speed
    final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        //TODO LOOK AT Generated version -- .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-cen
    
    SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    Telemetry logger = new Telemetry(MaxSpeed);
    
    SwerveRequest.Idle idle = new SwerveRequest.Idle();

  // The robot's subsystems and commands are defined here...
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(3, 4);
  private final SparkMaxShooterSubsystem m_sparkShooterSubsystem = new SparkMaxShooterSubsystem(3, 4);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    portForwardLimelight();
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
            drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * PERCENT_SPEED) // Drive forward with
                                                                                              // negative Y (forward)
                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * PERCENT_SPEED) // Drive left with negative X (left)
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
    m_driverController.y().whileTrue(new SparkTestShooterCommand(m_sparkShooterSubsystem));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press TODO test
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);
      }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public void updateVisionOdometry() {
      if (UseLimeLightAprilTag) {    
          Results lastResult = LimelightHelpers.getLatestResults("limelight-front").targetingResults;

          if (lastResult.valid && lastResult.targets_Fiducials.length > 0 && lastResult.targets_Fiducials[0].fiducialID != 0) {
              var alliance = DriverStation.getAlliance();

              if (alliance.isPresent()) {
                  if (alliance.get() == DriverStation.Alliance.Red) {
                      drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiRed(), Timer.getFPGATimestamp());
                  } else if (alliance.get() == DriverStation.Alliance.Blue){
                      drivetrain.addVisionMeasurement(lastResult.getBotPose2d_wpiBlue(), Timer.getFPGATimestamp());
                  }
              }
          }
      }
  }

  public void portForwardLimelight() {
      for (int limeLightPort = 5800; limeLightPort <= 5807; limeLightPort++) {
          int pcPort = limeLightPort + 0;
          PortForwarder.add(pcPort, "limelight-front.local", limeLightPort);
      }
  }
}
