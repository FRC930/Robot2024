package frc.robot.utilities;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/**
 * Gives the robot an odometry position when it starts in teleop if it doesn' already have a position
 */
public class StartInTeleopUtility {
    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private Consumer<Pose2d> m_ResetPose;

    private Pose2d temp;
    private Pose2d pose;

    private boolean m_HasRunAutonomous = false;
    private boolean m_HasSeenTags = false;

    private Trigger m_checkIfAllianceChangedTrigger = null;

    private boolean m_isFirstTime = true;

    /**
     * Resets the robots position.
     * @param resetPose
     */
    public StartInTeleopUtility(Consumer<Pose2d> resetPose) {
        m_ResetPose = resetPose;
    }

    /**
     * Lets us know that we have run in autonomous
     */
    public void updateAutonomous() {
        m_HasRunAutonomous = true;
    }

    /**
     * lets us know that we have seen april tags
     */
    public void updateTags() {
        m_HasSeenTags = true;
    }
    
    /**
     *Sets the robot position based on the alliance if there is one.
     */
    public void setRobotPositionBasedOnAlliance() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                temp = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();//.rotateBy(Rotation2d.fromDegrees(180.0));
                pose = new Pose2d(temp.getX() - 2.0, temp.getY(), temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
            } else {
                temp = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
                pose = new Pose2d(temp.getX() + 2.0, temp.getY(), temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
            }
            
            m_ResetPose.accept(pose);
        }
    }

    /**
     * Create a trigger and waits to update the alliance position when it is available
     */
    public void createTriggerForSimulation() {
        if(m_checkIfAllianceChangedTrigger == null) {
            m_checkIfAllianceChangedTrigger = new Trigger(()-> checkIsAlliancePresent())
                .onTrue((new InstantCommand(() -> setRobotPositionBasedOnAlliance())).ignoringDisable(true));
        }
    }

    /**
     * 
     * Returns the alliance
     * 
     * @return
     */
    private boolean checkIsAlliancePresent() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (m_isFirstTime) {
            m_isFirstTime = false;
            return false;
        }
        return optionalAlliance.isPresent();
    }

    /**
     * If auto hasn't been run and the robot hasn't seen april tags, it will then update the odoemtry when it enters teleop
     */
    public void updateStartingPosition() {
        if (m_HasRunAutonomous == false && m_HasSeenTags == false) {
            if(Robot.isReal()) {
                setRobotPositionBasedOnAlliance();
            } else {
                createTriggerForSimulation();
            }
        }
    }
}
