package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotOdometryUtility extends Command{
    private static final RobotOdometryUtility sim = new RobotOdometryUtility();

    private Pose2d m_pose;

    private RobotOdometryUtility() {
        m_pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    };

    public static RobotOdometryUtility getInstance() {
        return sim;
    };

    public void setRobotOdometry(Pose2d pose) {
        m_pose = pose;
    }

    public Pose2d getRobotOdometry() {
        return m_pose;
    }

}
