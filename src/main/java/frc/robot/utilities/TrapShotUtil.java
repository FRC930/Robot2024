package frc.robot.utilities;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Geometry2D;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class TrapShotUtil {
    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static double TrapShotDistance = 1.0;
    public static double TrapShotOffset = 0.0;

    private static Map<Integer, PathPlannerPath> stagePathingCommands = new HashMap<Integer, PathPlannerPath>();
    private static final int[] tagIDs = {11,12,13,14,15,16};
    private static final int[] redTagIDs = {11,12,13};
    private static final int[] bluTagIDs = {14,15,16};

    private static final double TRAP_SHOT_DISTANCE_TOLERANCE = 0.1;

    public static double TrapShotMaxSpeed = 5.861;
    public static double TrapShotMaxAccel = 5.0;
    public static double TrapShotMaxAngularSpeed = 5.0;
    public static double TrapShotMaxAngularAccel = 5.0;

    private static Alliance allianceInitializedWith = null;
    /**
     * Searches the speaker's apriltags to find which one is closest to the robotPose
     * @return
     */

    public static Integer getClosestTagID() {
        Pose2d robotPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        int[] aprilTagPoseIds;
        aprilTagPoseIds = tagIDs;

        double currentClosestDistance = -1;
        Integer bestTag = null;

        for(int i : aprilTagPoseIds) {
            double distance = field.getTagPose(i).get().toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if(distance < currentClosestDistance || currentClosestDistance == -1) {
                currentClosestDistance = distance;
                bestTag = i;
            }
        }

        return bestTag;
    }

    public static Integer getClosestAllianceTagID() {
        Pose2d robotPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        int[] aprilTagPoseIds;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        aprilTagPoseIds = alliance == Alliance.Blue ? bluTagIDs : redTagIDs;

        double currentClosestDistance = -1;
        Integer bestTag = null;

        for(int i : aprilTagPoseIds) {
            double distance = field.getTagPose(i).get().toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if(distance < currentClosestDistance || currentClosestDistance == -1) {
                currentClosestDistance = distance;
                bestTag = i;
            }
        }

        return bestTag;
    }


    public static void initStagePathingCommands(Alliance alliance) {
        stagePathingCommands = new HashMap<Integer, PathPlannerPath>();
        allianceInitializedWith = alliance;
        int[] aprilTagPoseIds;
        aprilTagPoseIds = tagIDs;

        for(int i : aprilTagPoseIds) {
            stagePathingCommands.put(Integer.valueOf(i),getPathToTrapShotPos(i));
        }
    }

    public static Pose2d movePoseByDist(Pose2d pose, double distance, double sideOffset) {
        Translation2d translationPose = new Translation2d(distance, sideOffset);
       
        Pose2d newPose = pose.transformBy(new Transform2d(translationPose,new Rotation2d()));
        return newPose;
    }

    public static Pose2d getTargetStagePose(Integer targetID) {
        return movePoseByDist(field.getTagPose(targetID).get().toPose2d(),TrapShotDistance,TrapShotOffset);
    }

    private static PathPlannerPath getPathToTrapShotPos(Integer poseId) {
        Pose2d targetPose = getTargetStagePose(poseId);
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.bezierFromPoses(RobotOdometryUtility.getInstance().getRobotOdometry(),targetPose),
                new PathConstraints(TrapShotMaxSpeed, TrapShotMaxAccel, TrapShotMaxAngularSpeed, TrapShotMaxAngularAccel),
                new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;
        path.flipPath();
        return path;
            
    }

    public static Command getPathtoClosestTrapShot(SwerveDrivetrainSubsystem swerve) {
        
        return new ProxyCommand(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            if(alliance != allianceInitializedWith || stagePathingCommands.isEmpty()) {
                initStagePathingCommands(alliance);
            }
            PathPlannerPath path = stagePathingCommands.get(getClosestAllianceTagID())
                .replan(RobotOdometryUtility.getInstance().getRobotOdometry(), swerve.getCurrentRobotChassisSpeeds());
            path.preventFlipping = true;
            return AutoBuilder.followPath(path);
            }
        );
    }

    // public static Command getWaitUntilAtTrapShotPos(double timeout) {
    //     return new ProxyCommand(() -> {
    //         if(alliance != allianceInitializedWith || stagePathingCommands.isEmpty()) {
    //             initStagePathingCommands(alliance);
    //         }
    //         Translation2d target = getTargetStagePose(getClosestAllianceTagID(alliance)).getTranslation();
    //         Debouncer boolDB = new Debouncer(0.1,DebounceType.kBoth);
    //         return new WaitUntilCommand(() -> {
    //             return boolDB.calculate(RobotOdometryUtility.getInstance().getRobotOdometry().getTranslation().getDistance(target) < TRAP_SHOT_DISTANCE_TOLERANCE);
    //         })
    //         .raceWith(new WaitCommand(timeout));
    //     });
    // }
}
