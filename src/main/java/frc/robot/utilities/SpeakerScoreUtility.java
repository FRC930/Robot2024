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
import frc.robot.subsystems.turret.TurretSubsystem;

public class SpeakerScoreUtility {

    // {LeftShooterSpeed, RightShooterSpeed, IndexerSpeed, PivotPosition, ElevatorPosition}
    static final private int LEFT_SPEED_COLUMN = 0;
    static final private int RIGHT_SPEED_COLUMN = 1;
    static final private int PIVOT_ANGLE_COLUMN = 3;

    static final private double FIXED_ANGLE_BUMPER_SHOT_DISTANCE = 43.0; //In inches
    private static final double FIXED_ANGLE_BUMPER_SHOT = 56.0;

    private final static double LINEAR_DISTANCE_FAR = 132.0 - 4.0; //subracting 4 inches to make sure 11 feet uses linear
    private static final double LINEAR_DISTANCE_CLOSE = 84.0 - 4.0; //subracting 4 inches to make sure 7 feet uses linear

    static final private double COMPUTED_SHOOT_SPEED = 117.0;

    static final private int CLOSE_ROW = 0;
    static final private int MEDIUM_ROW = 1;
    static final private int FAR_ROW = 2;
    private final static double[][] SHOOTING_CONSTANTS = 
    {
        {135.0, 135.0, 0.9, FIXED_ANGLE_BUMPER_SHOT, 0.0}, // "close" bumper shot
        {135.0, 135.0, 0.9, computePivotAngle(100.0), 0.0}, // "medium" 100 inches in line with podium
        {135.0, 135.0, 0.9, 27.45, 0.0} // "far" 185 inches back bumpers against pillar
    };
    private static final double DISTANCE_OFFSET_TO_CENTER_OF_ROBOT = 11.5;

    private static final double RED_ALLIANCE_OFFSET = 0.0; //was 2.0
    private static final double BLUE_ALLIANCE_OFFSET = 0.0; //was 2.0
    private static double m_nextShotAngleOffset = 0.0;
    private static double m_nextShotSpeedOverride;

    private boolean m_useAutoAim = true;

    private final TurretSubsystem m_turret;

    public enum Target{
        close,
        medium,
        far
    };

    private Target m_desiredTarget;
    private static AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public SpeakerScoreUtility(TurretSubsystem turret) {
        m_desiredTarget = Target.close;
        m_turret = turret;

    }

    public void setDesiredTarget(Target target) {
        if (m_useAutoAim) {
            m_useAutoAim = false;
            m_turret.enableTurretLock();
        } else if (m_desiredTarget == target) {
            m_useAutoAim = true;
            m_turret.disableTurretLock();
        }
        Logger.recordOutput(this.getClass().getSimpleName()+"/UseAutoAim", m_useAutoAim);
        m_desiredTarget = target;
        Logger.recordOutput(this.getClass().getSimpleName()+"/DesiredTarget", target.toString());
    }

    public boolean getAutoAim() {
        return m_useAutoAim;
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
        return new InstantCommand(() -> {
            setDesiredTarget(desiredTarget);
            m_turret.getCurrentCommand().cancel();
        });
    }

    public double getLeftShooterSpeed() {                
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][LEFT_SPEED_COLUMN];
    }

    public double getRightShooterSpeed() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][RIGHT_SPEED_COLUMN];
    }

    public double getPivotAngle() {
        return SHOOTING_CONSTANTS[getRowForDesiredTarget()][PIVOT_ANGLE_COLUMN];
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
        return inchesToSpeaker(false, null, null);
    }

    public static double inchesToSpeaker(boolean useProxyPose, Pose2d redPose, Pose2d bluePose) {
        Pose2d m_RedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        Pose2d m_BlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        Pose2d m_TargetPose = m_BlueTargetPose;
        Pose2d m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();

        // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
            Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = m_RedTargetPose;
            } else {
                m_TargetPose = m_BlueTargetPose;
            }
            if (useProxyPose) m_CurrentPose = (alliance == Alliance.Red ? redPose : bluePose);
        }

        double distance = Units.metersToInches(Math.hypot(m_TargetPose.getX() - m_CurrentPose.getX(), m_TargetPose.getY() - m_CurrentPose.getY()))
             - DISTANCE_OFFSET_TO_CENTER_OF_ROBOT;
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distance", distance);
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/distanceFeet", distance /12.0);
        return distance;
    }

    public static double computePivotAngle(double distance) {
        return computePivotAnglePolyModel(distance) + m_nextShotAngleOffset;
    }

    /*  SHOOTER DATA SHEET HERE:
    // SEE INSTRUCTION SHEET: https://docs.google.com/document/d/1Bgt9aPj9Tyn7bf3AqmL5ytYnkF5hi1Utf2yvQWLiGkc/edit?usp=sharing
    // OPTIONS FOR TUNING:
    // 1. You can add more data points for the regression. Make sure to update the link though! The link to the calculator will change after saving it.
    // 2. Use piecewise overrides or modifiers for areas that don't work
    */
    public static double computePivotAnglePolyModel(double distance) {
        double exponent = 0.982502;
        double h = 230.316 + AimingMathUtil.DIST_FUDGE + SmartDashboard.getNumber("offsets/distanceOffset",  0.0);
        double k = 23.5 + 0.5; //25.9504 - old value before champs;

        if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            h += AimingMathUtil.BLUE_DIST_FUDGE;
            k += AimingMathUtil.BLUE_ANGLE_FUDGE;
        } else {
            h += AimingMathUtil.RED_DIST_FUDGE;
            k += AimingMathUtil.RED_ANGLE_FUDGE; 
        }

        return Math.pow(exponent , (distance - h)) + k;
    }

    public static double computeShooterSpeed(double distance) {
        return (m_nextShotSpeedOverride > 0.0)?m_nextShotSpeedOverride:COMPUTED_SHOOT_SPEED;
    }

    public static void setShotOffset(double angleOffset) {
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/shotOffset",angleOffset);
        m_nextShotAngleOffset = angleOffset;
    }
    
    public static void resetShotOffset() {
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/shotOffset",0.0);
        m_nextShotAngleOffset = 0.0;
    }

    public static void setShotSpeedOffset(double speedOverride) {
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/shotSpeedOverride",speedOverride);
        m_nextShotSpeedOverride = speedOverride;
    }
    
    public static void resetShotSpeedOffset() {
        Logger.recordOutput(SpeakerScoreUtility.class.getSimpleName() + "/shotSpeedOverride",0.0);
        m_nextShotSpeedOverride = 0.0;
    }
}