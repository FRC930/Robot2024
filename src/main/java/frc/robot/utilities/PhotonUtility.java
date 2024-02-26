package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonUtility {
    private static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); 
    public enum PhysPhotonCamera {
        FrontLeft("front-left", new Translation3d(), new Rotation3d()),
        FrontRight("front-right", new Translation3d(), new Rotation3d()),
        BackLeft("back-left", new Translation3d(), new Rotation3d()),
        BackRight("back-right", new Translation3d(), new Rotation3d());

        private String name;
        private PhotonCamera camera;
        private PhotonPoseEstimator photonPoseEstimator;
        protected Transform3d cameraPos;
        PhysPhotonCamera(String name, Translation3d translation, Rotation3d rotation) {
            this.name = name; 
            this.cameraPos = new Transform3d(translation, rotation);
            this.camera = new PhotonCamera(name);
            this.photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, this.cameraPos);
        } 

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            this.photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return this.photonPoseEstimator.update();
        }
    }

    private static final PhysPhotonCamera[] cameras = {PhysPhotonCamera.FrontLeft, PhysPhotonCamera.FrontRight, PhysPhotonCamera.BackLeft, PhysPhotonCamera.BackRight};
    public static List<EstimatedRobotPose> getEstimatedPoses(Pose2d prevEstimatedRobotPose) {
        List<EstimatedRobotPose> poses = new ArrayList<>();
        for(PhysPhotonCamera camera : cameras) {
            Optional<EstimatedRobotPose> pose = camera.getEstimatedGlobalPose(prevEstimatedRobotPose);
            if(pose.isPresent()) {
                EstimatedRobotPose estimatedPose = pose.get();
                poses.add(
                    new EstimatedRobotPose(
                        estimatedPose.estimatedPose.transformBy(camera.cameraPos),
                        estimatedPose.timestampSeconds,
                        estimatedPose.targetsUsed,
                        estimatedPose.strategy
                    )
                );
            }
        }

        return poses;
    }
}
