package frc.robot.utilities;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class StartInTeleopUtility {
    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private Consumer<Pose2d> m_ResetPose;

    private Pose2d temp;
    private Pose2d pose;

    private boolean m_HasRunAutonomous = false;
    private boolean m_HasSeenTags = false;

    private Trigger m_checkIfAllianceChangedTrigger = null;

    private boolean m_isFirstTime = false;

    public StartInTeleopUtility(Consumer<Pose2d> resetPose) {
        m_ResetPose = resetPose;
    }

    public void updateAutonomous() {
        m_HasRunAutonomous = true;
    }

    public void updateTags() {
        m_HasSeenTags = true;
    }
    
    public void setRobotPositionBasedOnAlliance() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                temp = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
                pose = new Pose2d(temp.getX() - 2.0, temp.getY(), temp.getRotation());
            } else {
                temp = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
                pose = new Pose2d(temp.getX() + 2.0, temp.getY(), temp.getRotation());
            }
            m_ResetPose.accept(pose);

        }
    }

    public void createTriggerForSimulation() {
        if(m_checkIfAllianceChangedTrigger == null) {
            m_checkIfAllianceChangedTrigger = new Trigger(()-> checkIsAlliancePresent())
                .onTrue((new InstantCommand(() -> setRobotPositionBasedOnAlliance())).ignoringDisable(true));
        }
    }

    private boolean checkIsAlliancePresent() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (m_isFirstTime) {
            m_isFirstTime = false;
            return false;
        }
        return optionalAlliance.isPresent();
    }

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
