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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TrapShotUtil {
    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static double TrapShotDistance = 0.5;
    public static double TrapShotOffset = 0.2;

    private static Map<Integer, PathPlannerPath> stagePathingCommands = new HashMap<Integer, PathPlannerPath>();
    private static final int[] tagIDs = {11,12,15,16};
    private static final int[] redTagIDs = {11,12};
    private static final int[] bluTagIDs = {15,16};

    private static final double TRAP_SHOT_DISTANCE_TOLERANCE = 0.1;

    private static final double TRAP_SHOT_ANGLE_TOLERANCE = 0.1;

    public static double TrapShotMaxSpeed = 5.861;
    public static double TrapShotMaxAccel = 3.0;
    public static double TrapShotMaxAngularSpeed = 50.0;
    public static double TrapShotMaxAngularAccel = 50.0;

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
                PathPlannerPath.bezierFromPoses(new Pose2d(),new Pose2d(targetPose.getX(),targetPose.getY(),targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)))),
                new PathConstraints(TrapShotMaxSpeed, TrapShotMaxAccel, TrapShotMaxAngularSpeed, TrapShotMaxAngularAccel),
                new GoalEndState(0, targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
        path.preventFlipping = true;
        return path;
            
    }

    public static Command getPathtoClosestTrapShot() {
        return new ProxyCommand(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
            if(alliance != allianceInitializedWith || stagePathingCommands.isEmpty()) {
                initStagePathingCommands(alliance);
            }
            PathPlannerPath path = stagePathingCommands.get(getClosestAllianceTagID());
            path.preventFlipping = true;
            return AutoBuilder.followPath(path);
            }
        );
    }

    public static boolean getAtTrapPosition() {
        Pose2d currentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        Pose2d _targetPose = getTargetStagePose(getClosestAllianceTagID());
        Pose2d finalPose = new Pose2d(_targetPose.getX(),_targetPose.getY(),_targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        Debouncer db = new Debouncer(0.5);

        return db.calculate(
            MathUtil.isNear(currentPose.getX(), finalPose.getX(), TRAP_SHOT_DISTANCE_TOLERANCE) 
            && MathUtil.isNear(currentPose.getX(), finalPose.getX(), TRAP_SHOT_DISTANCE_TOLERANCE)
            && MathUtil.isNear(currentPose.getRotation().getDegrees(), finalPose.getRotation().getDegrees(), TRAP_SHOT_ANGLE_TOLERANCE));
        
    }

    public static Command getPathToClosestTrapShotAndRepeat(double timeout) {
        return new RepeatCommand(getPathtoClosestTrapShot())
            .raceWith(new WaitUntilCommand(() -> {return getAtTrapPosition();})
                .raceWith(new WaitCommand(timeout))
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
