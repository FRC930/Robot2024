package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class AprilTagUtility {
    SwerveDrivetrainSubsystem drivetrain = TunerConstants.DriveTrain;
    private StartInTeleopUtility m_StartInTeleopUtility = new StartInTeleopUtility(drivetrain::seedFieldRelative);

    private final boolean USE_LIMELIGHT_APRIL_TAG = true;
    public boolean m_visionUpdatesOdometry = true;

    private int visioncounter = 0;

    public AprilTagUtility() {

    }

    /**
     * Update all vision
     */
    public void updateAllVision() {
        if (USE_LIMELIGHT_APRIL_TAG) {
            // updatePoseEstimateWithAprilTags("limelight-front",true);
            // updatePoseEstimateWithAprilTags("limelight-back",true);
            // updatePoseEstimateWithAprilTags("limelight-right", true);
            // updatePoseEstimateWithAprilTags("limelight-left", true);
            updatePoseWithMegaTag2("limelight-front", true);
            updatePoseWithMegaTag2("limelight-back", true);
            updatePoseWithMegaTag2("limelight-right", true);
            updatePoseWithMegaTag2("limelight-left", true);
        }
    }

    // Add the rotation speed to reject positions
    public void updatePoseWithMegaTag2(String limeLightName, boolean usePose) {
        boolean doRejectUpdate = false;
        double fpgaTimestamp = Timer.getFPGATimestamp();

        LimelightHelpers.SetRobotOrientation(limeLightName,
                RobotOdometryUtility.getInstance().getRobotOdometry().getRotation().getDegrees(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);

        double rotationalRate = TunerConstants.DriveTrain.getPigeon2().getRate();
        Logger.recordOutput("LimeLightOdometry/rotationalRate", rotationalRate);
        if (Math.abs(rotationalRate) >= 200)
        {
            doRejectUpdate = true;
        }

        // distance from current pose to vision estimated pose
        // Translation2d translation =
        // TunerConstants.DriveTrain.getState().Pose.getTranslation();
        // double poseDifference = translation.getDistance(mt2.pose.getTranslation());

        double xyStds;
        // if (mt2.tagCount >= 2) {
        xyStds = 0.2;
        // }

        // 1 target with large area and close to estimated pose
        // else if (mt2.tagCount == 1 && mt2.avgTagArea > 0.8 && poseDifference < 0.5) {
        // xyStds = 1.0;
        // degStds = 12;
        // }
        // conditions don't match to add a vision measurement
        // else {
        // SmartDashboard.putBoolean(limeLightName + "/Updated", false);
        // return;
        // }

        if (m_visionUpdatesOdometry && usePose && !doRejectUpdate && mt2.tagCount > 0) {
            m_StartInTeleopUtility.updateTags();

            drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 99999.0));
            drivetrain.addVisionMeasurement(mt2.pose, fpgaTimestamp - (mt2.timestampSeconds / 1000.0));

            SmartDashboard.putBoolean(limeLightName + "/Updated", true);
            Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/UpdatCounts", this.visioncounter);
            this.visioncounter++;
        }

        Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", mt2.pose);
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

        if (m_visionUpdatesOdometry && usepose) {
            m_StartInTeleopUtility.updateTags();

            drivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

            drivetrain.addVisionMeasurement(lastResult.pose,
                    fpgaTimestamp - (lastResult.latency / 1000.0));

            SmartDashboard.putBoolean(limeLightName + "/Updated", true);
            Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/UpdatCounts", this.visioncounter);
            this.visioncounter++;
        }

        Logger.recordOutput("LimeLightOdometry/" + limeLightName + "/Pose", lastResult.pose);
        Logger.recordOutput("LimeLightOdometry/TA", lastResult.avgTagArea);
        Logger.recordOutput("LimeLightOdometry/PoseDifference", poseDifference);
    }
}
