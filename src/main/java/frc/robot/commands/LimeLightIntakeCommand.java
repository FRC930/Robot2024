package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.utilities.LimeLightDetectionUtility;
import frc.robot.utilities.LimelightHelpers;
/**
 * 
 * <h3>LimeLightIntakeCommand</h3>
 * 
 * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a game pieces during autonomous
 * 
 */
public class LimeLightIntakeCommand extends Command {
    private final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps;
    private final double MAX_STRAFE = 0.3; 
    private final double MAX_THROTTLE = 1.0;
    private final double MOVING_AVERAGE_SECONDS = 0.1;
    private final double HIGH_PASS_SECONDS = 1.0;
    private final double HIGH_PASS_LIMIT = 0.1;
    private PIDController pid = new PIDController(0.0065, 0.0, 0.0);

    private SwerveDrivetrainSubsystem m_SwerveDrive;
    private LimeLightDetectionUtility m_LimeLight;

    private Pose2d m_bluePosition;
    private Pose2d m_redPosition;
    private Pose2d m_position;
    
    private double m_throttle = 0.0;
    private CommandXboxController m_controller;
    private double m_strafe = 0.0;
    private double slope = 0.9; //Adjust if the location of the game piece camera moves

    private double m_distance;  
    private double m_TimeElapsed;

    private final TrapezoidProfile.Constraints m_constraints = //maximum velocity and acceleration for the command
        new TrapezoidProfile.Constraints(MAX_SPEED, 1);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile profile;

    private SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private double m_direction;

    private final double BUFFER_NOTE_X = -0.2;

    private LinearFilter movingAverageFilter;
    // private LinearFilter highPassFilter;

    /**
     * <h3>LimeLightIntakeCommand</h3>
     * 
     * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
     * 
     * @param swerveDrive Swerve Drive
     * @param limeLight LimeLightDetectionUtility
     * @param bluePosition Pose2d of the location of where the robot should go
     * 
     */
    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight, Pose2d bluePosition, Pose2d redPosition, CommandXboxController controller) {
        m_SwerveDrive = swerveDrive;
        m_LimeLight = limeLight;
        m_bluePosition = bluePosition;
        m_redPosition = redPosition;
        // Default to blue alliance
        m_position = m_bluePosition;
        m_controller = controller;
        addRequirements(m_SwerveDrive);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight, Pose2d bluePosition, Pose2d redPosition) {
        this(swerveDrive, limeLight, bluePosition, redPosition, null);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem drivetrain, LimeLightDetectionUtility m_GamePieceUtility, CommandXboxController controller) {
        this(drivetrain, m_GamePieceUtility, null, null, controller);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight,Pose2d pose2d) {
        this(swerveDrive, limeLight, pose2d, pose2d, null);
    }

    @Override
    public void initialize() { 
        m_direction = -1.0; //Currently camera in back of robot (go backwards) when camera in front use 1.0
        //Default to blue alliance (named commands are reused)
        m_position = m_bluePosition;

        // Force pipeline zero to see if switches to detector (MODE)
        LimelightHelpers.setPipelineIndex(m_LimeLight.m_LimeLightName, 0);

        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();

        //if on red alliance, return as negative
        //if on blue alliance, return as positive
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_position = m_redPosition;
                // Move point forward for red alliance (auto)
                if(m_position != null) {
                    m_position = new Pose2d(m_position.getX()-BUFFER_NOTE_X, m_position.getY(), m_position.getRotation());
                }
            } else {
                if(m_position != null) {
                    // Move point forward for blue alliance (auto)
                    m_position = new Pose2d(m_position.getX()+BUFFER_NOTE_X, m_position.getY(), m_position.getRotation());
                }
            }
        }

        m_TimeElapsed = 0.0;

        movingAverageFilter = LinearFilter.movingAverage(3);
        if(m_controller != null) {
           return;
        }

        m_distance = distanceToTarget();
        
        //Creates the trapezoid profile using the given information
        m_goal = new TrapezoidProfile.State(m_distance, 0.0); //sets the desired state to be the total distance away
        m_setpoint = new TrapezoidProfile.State(0.0, 2.0); //sets the current state at (0,0)
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint); //combines everything into the trapezoid profile
        
        // highPassFilter = LinearFilter.highPass(HIGH_PASS_SECONDS, 0.02);
    }

    @Override
    public void execute() {
        // Crosshair isn't in the exact center, but instead to where the note will enter the robot
        double tx = m_LimeLight.get_tx(); // degrees left and right from crosshair
        double ty = m_LimeLight.get_ty(); // degrees up and down from crosshair
        double linearTX = movingAverageFilter.calculate(tx <= -100.0 && ty <= -100.0? 0.0 : (ty/slope) - tx + 2.0); //-100.0 is just a temporary value that cannot be reached
        
        /** 
         * Since our camera isn't centered on the robot, when a note moves forward/backwards it will subsequently move left/right
         * linearTX will automatically see how far forwards/backwards the note is and determine how many degrees off is the actual center using a linear slope
        */
        if (tx == 0.0 || ty == 0.0) {
            linearTX = 0.0;
        }

        // Prevents the robot from unwantingly jumped to a new note
        // if (Math.abs(highPassFilter.calculate(linearTX)) > HIGH_PASS_LIMIT) {
        //     linearTX = 0.0;
        // }

        //uses a clamp and pid on the game piece detection camera to figure out the strafe (left & right)
        m_strafe = m_direction * MathUtil.clamp(pid.calculate(-linearTX, 0.0), -MAX_STRAFE, MAX_STRAFE) * MAX_SPEED; 

        if(m_controller != null) {
            double xValue = Math.abs(Math.hypot(m_controller.getLeftX(), m_controller.getLeftY()));  // -negated value since back intake so need to backward
            xValue = MathUtil.clamp(xValue, -MAX_THROTTLE, MAX_THROTTLE);
            m_throttle = RobotContainer.scaleLinearVelocity(RobotContainer.getLinearVelocity(xValue, 0.0).getX());
        } else {
            m_throttle =  m_direction * profile.calculate(m_TimeElapsed).velocity; //sets the throttle (speed) to  the current point on the trapezoid profile
        } 
        
        Logger.recordOutput("GamePiece/TX", tx);
        Logger.recordOutput("GamePiece/TY", ty);
        Logger.recordOutput("GamePiece/adjustedTX", linearTX);
        Logger.recordOutput("GamePiece/throttle", m_throttle);
        Logger.recordOutput("GamePiece/strafe", m_strafe);
        m_TimeElapsed += 0.02; //increases the timer  by 20 milliseconds

        /* 
         * This command utilitzes the swerve drive while it isn't field relative. 
         * The swerve drive returns back to field relative after the command is used.
        */

        Supplier<SwerveRequest> requestSupplier = () -> forwardStraight.withVelocityX(m_throttle).withVelocityY(m_strafe);
        m_SwerveDrive.setControl(requestSupplier.get());
    }

    @Override
    public boolean isFinished() {
        if (m_controller != null) {
            return false;
        }
        if (profile.isFinished(m_TimeElapsed)) {
            return true;
        } else {
            return false;
        }
    }

    public double distanceToTarget() {
        // m_position only popuated if not using joystick input and this distanceToTarget() should not called, but just in case add check
        if(m_position != null) {
            return Math.sqrt( //Uses the Pythagorean Theorem to calculate the total distance to the target
                Math.pow(
                    Math.abs(m_position.getX() - m_SwerveDrive.getState().Pose.getX()), 
                    2.0
                )
                +
                Math.pow(
                    Math.abs(m_position.getY() - m_SwerveDrive.getState().Pose.getY()),
                    2.0
                )
            );
        } else {
            return 0.0;
        }
    }
}