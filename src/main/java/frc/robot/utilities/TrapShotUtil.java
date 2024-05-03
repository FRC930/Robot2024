package frc.robot.utilities;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TrapShotUtil {
    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    /**
     * Searches the speaker's apriltags to find which one is closest to the robotPose
     * @return
     */
    public static Pose2d getClosestTag(Pose2d robotPose,Alliance alliance) {
        
        int[] aprilTagPoseIds;
        if(alliance == Alliance.Red) {
           aprilTagPoseIds = new int[] {11,12,13};
        } else {
           aprilTagPoseIds = new int[] {14,15,16}; 
        }

        double currentClosestDistance = -1;
        Pose2d newPose = null;

        for(int i : aprilTagPoseIds) {
            Pose2d pose = field.getTagPose(i).get().toPose2d();
            double distance = pose.getTranslation().getDistance(robotPose.getTranslation());
            if(distance < currentClosestDistance || currentClosestDistance == -1) {
                currentClosestDistance = distance;
                newPose = pose;
            }
        }

        return newPose;
    }

    public static Pose2d movePoseByDist(Pose2d pose, double distance) {
        Translation2d translationPose = new Translation2d(distance, 0.0);
       
        Pose2d newPose = pose.transformBy(new Transform2d(translationPose,new Rotation2d()));
        return newPose;
    }

    public static Pose2d getTargetStagePose(Pose2d robotPose,double distance,Alliance alliance) {
        return movePoseByDist(getClosestTag(robotPose,alliance),distance);
    }

    public static Pose2d getTargetStagePose(Pose2d robotPose,double distance) {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        Alliance setAlliance = null;
        if(optionalAlliance.isPresent()){
            setAlliance = optionalAlliance.get();
        } else {
            setAlliance = Alliance.Blue;
        }
        return getTargetStagePose(robotPose, distance, setAlliance);
    }
}
