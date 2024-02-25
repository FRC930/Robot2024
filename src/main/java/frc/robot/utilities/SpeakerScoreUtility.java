package frc.robot.utilities;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpeakerScoreUtility {

    // {LeftShooterSpeed, RightShooterSpeed, IndexerSpeed, PivotPosition, ElevatorPosition}
    static final private int LEFT_SPEED_COLUMN = 0;
    static final private int RIGHT_SPEED_COLUMN = 1;
    static final private int INDEXER_SPEED_COLUMN = 2;  // TODO not needed since never used different value
    static final private int PIVOT_ANGLE_SPEED_COLUMN = 3;
    static final private int EVELATOR_HEIGHT_COLUMN = 4;   // TODO not need given all at 0.0 

    static final private double FIXED_ANGLE_BUMPER_SHOT_DISTANCE = 43.0; //In inches
    private static final double FIXED_ANGLE_BUMPER_SHOT = 56.0;

    private final static double LINEAR_DISTANCE_FAR = 132.0 - 4.0; //subracting 4 inches to make sure 11 feet uses linear
    private static final double LINEAR_DISTANCE_CLOSE = 84.0 - 4.0; //subracting 4 inches to make sure 7 feet uses linear

    static final private double COMPUTED_SHOOT_SPEED = 140.0;

    static final private int CLOSE_ROW = 0;
    static final private int MEDIUM_ROW = 1;
    static final private int FAR_ROW = 2;
    private final static double[][] SHOOTING_CONSTANTS = 
    {
        {140.0, 140.0, 0.9, 40.0, 0.0}, // "close" 3' 7" from speaker bumper to front of frame
        {140.0, 140.0, 0.9, 33.0, 0.0}, // "medium" 7' 8" from speaker bumper to front of frame
        {140.0, 140.0, 0.9, 31.0, 0.0} // "far" 12' 7" from speaker bumper to front of frame (ONLY WORKS WITH UNTRIMMED NOTES)
    };
    private static final double DISTANCE_OFFSET_TO_CENTER_OF_ROBOT = 11.5;
    


    public enum Target{
        close, // 3' 7" from speaker bumper to front of frame
        medium, // 7' 8" from speaker bumper to front of frame
        far // 12' 7" from speaker bumper to front of frame (ONLY WORKS WITH UNTRIMMED NOTES)
    };

    private Target m_desiredTarget;

    public SpeakerScoreUtility() {
        m_desiredTarget = Target.close;
    }

    public void setDesiredTarget(Target target) {
        m_desiredTarget = target;
        SmartDashboard.putString(this.getClass().getSimpleName()+"/DesiredTarget", target.toString());
    }

    public Target getDesiredTarget() {
        return m_desiredTarget;
    }

    public boolean isClose() {
        return m_desiredTarget == Target.close;
    }

    public boolean isMedium() {
        return m_desiredTarget == Target.medium;
    }

    public boolean isFar() {
        return m_desiredTarget == Target.far;
    } 

    public Command setDesiredTargetCommand(Target desiredTarget) {
        return new InstantCommand(() -> setDesiredTarget(desiredTarget));
    }

    public double getLeftShooterSpeed() {                
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][LEFT_SPEED_COLUMN];
    }

    public double getRightShooterSpeed() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][RIGHT_SPEED_COLUMN];
    }

    public double getPivotAngle() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][PIVOT_ANGLE_SPEED_COLUMN];
    }

    private int getRowForDesiredTarget() {
        switch(m_desiredTarget) {
            case close:
                return CLOSE_ROW;
            case medium:
                return MEDIUM_ROW;
            case far:
                return FAR_ROW;
            default:
                return CLOSE_ROW;
        }
    }

    public static double inchesToSpeaker() {
        AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        Pose2d m_RedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        Pose2d m_BlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        Pose2d m_TargetPose = m_BlueTargetPose;

        // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = m_RedTargetPose;
            } else {
                m_TargetPose = m_BlueTargetPose;
            }
        }
        // gets the robots position, and gets the robots heading.
        Pose2d m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();

        double distance = Units.metersToInches(Math.hypot(m_TargetPose.getX() - m_CurrentPose.getX(), m_TargetPose.getY() - m_CurrentPose.getY()))
             - DISTANCE_OFFSET_TO_CENTER_OF_ROBOT;
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distance", distance);
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distanceFeet", distance /12.0);
        return distance;
    }

    public static double computePivotAngle(double distance) {
        double exponent = -0.152665;
        double h = 19.5834;
        double k = 19.5854;
        double angleOffset = 0.0; // 3.0;
        if(distance <= FIXED_ANGLE_BUMPER_SHOT_DISTANCE){
            return FIXED_ANGLE_BUMPER_SHOT;
        } else if (distance >= LINEAR_DISTANCE_FAR) {
            return (-0.05 * distance) + 32.7 + 0.5 + angleOffset; // 0.5 (inches) is a fudge factor
        } else if (distance >= LINEAR_DISTANCE_CLOSE) {
            return (-0.115 * distance) + 40.9 + 2.0 + angleOffset; // 2.0 (inches) is a fudge factor
        } else {
            return (1.95E-3 * Math.pow(distance, 2)) - (0.54 * distance) + 63.3 + 2.0 + angleOffset; // 2.0 (inches) is a fudge factor
        }
        // Untested shot angle model. Distances sourced from testing on 4/23. Source graph: https://www.desmos.com/calculator/me4nlqffa5
        // return Math.exp(exponent * (distance - 4 - h)) + k;
    }

    public static double computeShooterSpeed(double distance) {
        return COMPUTED_SHOOT_SPEED;
    }

}