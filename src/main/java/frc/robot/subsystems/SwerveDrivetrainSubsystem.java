package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.Notifier;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.RobotOdometryUtility;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
    private boolean hasAppliedOperatorPerspective = false;
    private boolean usePredictedPose = true;

    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.Velocity);
    private Field2d pp_field2d = new Field2d(); // TODO: Move to AutoCommandManager
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final double LOOKAHEAD = 0.1;

    public SwerveDrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public SwerveDrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void configurePathPlanner() {
        SmartDashboard.putData("pp_field", pp_field2d);
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose,
            this::seedFieldRelative,
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)),// Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(
                new PIDConstants(1.6, 0, 0), // TODO: Config
                new PIDConstants(7.0, 0, 0), // TODO: Config
                TunerConstants.kSpeedAt12VoltsMps, // Meters  // TODO get set to correct value
                Units.inchesToMeters(11.0), // TODO determine 
                new ReplanningConfig(),
                0.004),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        PathPlannerLogging.setLogTargetPoseCallback((Pose2d targetPose) -> {
            pp_field2d.setRobotPose(targetPose);
            Logger.recordOutput("PathPlanner/TargetPose", targetPose);
        });
    }
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    @Override
    public void periodic() {
        setDriverPerspective();

        // validates ModuleStaes are populated by odometry thread before logging them
        if (getState().ModuleStates != null) {
            Logger.recordOutput("Drivetrain/rotationVelocity", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
            Logger.recordOutput("Drivetrain/XVelocity", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
            Logger.recordOutput("Drivetrain/YVelocity", getCurrentRobotChassisSpeeds().vyMetersPerSecond);            
            Logger.recordOutput("Drivetrain/Pose", getState().Pose);

            Logger.recordOutput("Drivetrain/OmegaVelocityRadians", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);
            
            Logger.recordOutput("Drivetrain/Pigeon2Yaw", getPigeon2().getAngle());
            Logger.recordOutput("Drivetrain/Pose2DYaw", getState().Pose.getRotation().getDegrees());
            
            for (int i = 0; i < 4; i++) {
                Logger.recordOutput("Drivetrain/SwerveWheelSpeed/" + i, getState().ModuleStates[i].speedMetersPerSecond);
                Logger.recordOutput("Drivetrain/SwerveWheelAngle/" + i, getState().ModuleStates[i].angle.getDegrees());
            }
            //Calculate our predicted FUUUUUUTURE position 
            if(usePredictedPose) {
                Pose2d predictedPose = getState().Pose.transformBy(new Transform2d(new Translation2d(getCurrentRobotChassisSpeeds().vxMetersPerSecond * LOOKAHEAD, getCurrentRobotChassisSpeeds().vyMetersPerSecond * LOOKAHEAD),new Rotation2d(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond * LOOKAHEAD)));
                Logger.recordOutput("Drivetrain/PredictedPose", predictedPose);
                RobotOdometryUtility.getInstance().setRobotOdometry(predictedPose);
            } else {
                RobotOdometryUtility.getInstance().setRobotOdometry(getState().Pose);
            }
        }
    }

    private void setDriverPerspective() {
        //Got this code from CTRE
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
