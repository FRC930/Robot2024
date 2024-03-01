package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import frc.robot.utilities.LimeLightDetectionUtility;

/**
 * 
 * <h3>LimeLightIntakeCommand</h3>
 * 
 * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
 * 
 */
public class LimeLightIntakeCommand extends Command {
    private final double MAX_SPEED = Units.feetToMeters(16.2);
    private final double MAX_STRAFE = 0.2; //TODO tune this value on the robot. Tune PID value first and set this value as a hard stop to prevent outlying data
    private PIDController pid = new PIDController(0.01, 0.0, 0.0); //(0.01, 0.0, 0.0); //TODO tune this value

    private SwerveDrivetrainSubsystem m_SwerveDrive;
    private LimeLightDetectionUtility m_LimeLight;

    private Pose2d m_bluePosition;
    private Pose2d m_redPosition;
    private Pose2d m_position;
    
    private double m_throttle = 0.0;
    private Supplier<Double> m_joystickInput;
    private double m_strafe = 0.0;

    private double m_distance;  
    private double m_TimeElapsed;

    private final TrapezoidProfile.Constraints m_constraints = //maximum velocity and acceleration for the command
        new TrapezoidProfile.Constraints(MAX_SPEED, 1);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile profile;

    private SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
    private double m_direction;

    /**
     * <h3>LimeLightIntakeCommand</h3>
     * 
     * Uses game piece detection and a Trapezoidal Profile to smoothly and accurately intake a cube during autonomous
     * 
     * @param swerveDrive Swerve Drive
     * @param limeLight GamePieceDetectionUtility
     * @param bluePosition Pose2d of the location of where the robot should go
     * 
     */
    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight, Pose2d bluePosition, Pose2d redPosition, Supplier<Double> joystickInputSupplier) {
        m_SwerveDrive = swerveDrive;
        m_LimeLight = limeLight;
        m_bluePosition = bluePosition;
        m_redPosition = redPosition;
        // Default to blue alliance
        m_position = m_bluePosition;
        m_joystickInput = joystickInputSupplier;
        addRequirements(m_SwerveDrive);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight, Pose2d bluePosition, Pose2d redPosition) {
        this(swerveDrive, limeLight, bluePosition, redPosition, null);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem drivetrain, LimeLightDetectionUtility m_GamePieceUtility, Supplier<Double> joystickInputSupplier) {
        this(drivetrain, m_GamePieceUtility, null, null,joystickInputSupplier);
    }

    public LimeLightIntakeCommand(SwerveDrivetrainSubsystem swerveDrive, LimeLightDetectionUtility limeLight,Pose2d pose2d) {
        this(swerveDrive, limeLight, pose2d, pose2d, null);
    }
    
    // public LimeLightIntakeCommand(SwerveDrivetrainSubsystem drivetrain, LimeLightDetectionUtility m_GamePieceUtility, Pose2d pose2d) {
    //     this(drivetrain, m_GamePieceUtility, pose2d, (Supplier<Double>) null);
    // }

    @Override
    public void initialize() { 
        //TODO This assumes you are directly in front or behind (if overshoot will go wrong direction)
        m_direction = -1.0; //Currently camera in back of robot (go backwards) when camera in front use 1.0
        //Default to blue alliance (named commands are reused)
        m_position = m_bluePosition;

        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        //if on red alliance, return as negative
        //if on blue alliance, return as positive
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_position = m_redPosition;
            }
        }

        m_TimeElapsed = 0.0;

        if(m_joystickInput != null) {
           return;
        }

        m_distance = distanceToTarget();
        
        // SmartDashboard.putNumber("GamePiece/LimeLightDistance", m_distance);
        // SmartDashboard.putNumber("GamePiece/Xpos", m_SwerveDrive.getState().Pose.getX());
        // SmartDashboard.putNumber("GamePiece/Ypos", m_SwerveDrive.getState().Pose.getY());
        
        //Creates the trapezoid profile using the given information
        m_goal = new TrapezoidProfile.State(m_distance, 0.0); //sets the desired state to be the total distance away
        //TODO fix deprecated and how to get current velocity
        m_setpoint = new TrapezoidProfile.State(0.0, 1.0); //sets the current state at (0,0)
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint); //combines everything into the trapezoid profile
    }

    @Override
    public void execute() {

        //uses a clamp and pid on the game piece detection camera to figure out the strafe (left & right)
        m_strafe = m_direction * MathUtil.clamp(pid.calculate(m_LimeLight.get_tx(), 0.0), -MAX_STRAFE, MAX_STRAFE) * MAX_SPEED; 

        if(m_joystickInput != null) {
            double xValue = -m_joystickInput.get();
            m_throttle = RobotContainer.scaleLinearVelocity(RobotContainer.getLinearVelocity(xValue, 0.0).getX());
        } else {
            m_throttle =  m_direction * profile.calculate(m_TimeElapsed).velocity; //sets the throttle (speed) to  the current point on the trapezoid profile
        } 
        
        Logger.recordOutput("GamePiece/TX", m_LimeLight.get_tx());
        // SmartDashboard.putNumber("GamePiece/position", profile.calculate(m_TimeElapsed).position);
        // SmartDashboard.putNumber("GamePiece/throttle", m_throttle);
        // SmartDashboard.putNumber("GamePiece/strafe", m_strafe);
        // SmartDashboard.putNumber("GamePiece/distanceLeft", distanceToTarget());
        // SmartDashboard.putNumber("GamePiece/SteerVelocity", m_SwerveDrive.getModule(0).getSteerMotor().getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("GamePiece/DriveVelocity", m_SwerveDrive.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());

        m_TimeElapsed += 0.02; //increases the timer  by 20 milliseconds

        /* 
         * This command utilitzes the swerve drive while it isn't field relative. 
         * The swerve drive returns back to field relative after the command is used.
         * This is located in RobotContainer at line 155
        */


        Supplier<SwerveRequest> requestSupplier = () -> forwardStraight.withVelocityX(m_throttle).withVelocityY(m_strafe).withRotationalRate(0.0);
        m_SwerveDrive.setControl(requestSupplier.get());
    }

    @Override
    public boolean isFinished() {
        if (m_joystickInput != null) {
            return false;
        }
        if (profile.isFinished(m_TimeElapsed)) {
            // SmartDashboard.putNumber("GamePiece/ElapsedTime", m_TimeElapsed);
            // SmartDashboard.putNumber("GamePiece/EndXpos", m_SwerveDrive.getState().Pose.getX());
            // SmartDashboard.putNumber("GamePiece/EndYpos", m_SwerveDrive.getState().Pose.getY());
            return true;
        } else {
        // return MathUtil.applyDeadband(distanceToTarget(), 0.025, 1.0) == 0;
            return false; //ends the command when the timer reaches the end of the trapezoid profile
        }
    }

    public double distanceToTarget() {
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
    }
}

