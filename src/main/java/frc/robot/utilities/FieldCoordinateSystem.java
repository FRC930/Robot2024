package frc.robot.utilities;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class FieldCoordinateSystem {

    Supplier<Pose2d> m_getPoseMethod;
    private Consumer<Pose2d> m_resetPoseMethod;
    private Alliance m_lastAlliance = null;
    private Trigger m_checkIfAllianceChangedTrigger = null;
    private AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    /**
     * Assume if autopath run no need to re-set pose given used Auto had Starting Pose
     */
    private boolean m_runAutoPath = false;
    /**
     * Assume if apriltag use  no need to re-set pose given used Auto had Starting Pose
     */
    private boolean m_aprilTagUsed = false;
    private boolean m_firstTimeReturnFalse = true; // Needed for Trigger's onTrue to trigger

    

    public FieldCoordinateSystem(Supplier<Pose2d> getCurrentPoseMethod, Consumer<Pose2d> resetPoseMethod) {
        m_getPoseMethod = getCurrentPoseMethod;
        m_resetPoseMethod = resetPoseMethod;
    }


    // robot period when set apriltag
    public void odometryUpdatedWithAprilTag() {
        m_aprilTagUsed = true;
    }

    // get autocommand
    public void autoExecutedwithCommand(Command autonomousCommand) {
        if(autonomousCommand!=null) {
            m_runAutoPath = true;
            // OR if no starting pose in command??
        }
    }

    // Teleop init
    public void setRobotPoseOnField() {
        var a = whatisCurrentAlliance();
        if(Robot.isReal()) {
            m_firstTimeReturnFalse = false;  // needed for Trigger Ontrue
            if(checkIfAllianceChangedExpression()) {
                setRobotPoseOnFieldIfNoAprilTagOrAuto();
            }
        } else {
            createTriggerIfAllianceChanges();
        }
    }

    public void setRobotPoseOnFieldIfNoAprilTagOrAuto() {
        if(!m_aprilTagUsed && !m_runAutoPath) {
            Pose2d location;
            // Set default location where apriltag of speaker
            if(m_lastAlliance == Alliance.Red) {
                location = layout.getTagPose(4).get().toPose2d();
                location = new Pose2d(new Translation2d(location.getX()-2.0, location.getY()),location.getRotation());
            } else {
                location = layout.getTagPose(7).get().toPose2d();
                location = new Pose2d(new Translation2d(location.getX()+2.0, location.getY()),location.getRotation());
            }

            setRobotPoseBasedOnAlliance(location);
        }
    }

    private void setRobotPoseBasedOnAlliance(Pose2d location){
        m_resetPoseMethod.accept(location);
    }

    // Needed a trigger for simulation to work given alliance is not available in TeleopInit()
    private void createTriggerIfAllianceChanges() {        
        m_checkIfAllianceChangedTrigger = new Trigger(()->checkIfAllianceChangedExpression())
                                                .onTrue((new InstantCommand(() -> setRobotPoseOnFieldIfNoAprilTagOrAuto())).ignoringDisable(true));
    }

    private boolean checkIfAllianceChangedExpression() {
        boolean allianceChanged = false;
        if(m_firstTimeReturnFalse) {
            m_firstTimeReturnFalse = false;
            return false;

        }
        Alliance alliance = whatisCurrentAlliance();
        if(m_lastAlliance != null  && alliance != null) {
            allianceChanged = (alliance != m_lastAlliance);
        } else {
            if(alliance != null) {
                allianceChanged = true;
            }
        }
        // TODO how to reset (can not do here or would reset auto/april flag ever time start teleop)
        // if(m_lastAlliance != null && allianceChanged ) {
        //     /* reset state so pose will reset when using teleop */
        //     resetStateOfField();
        // }
        m_lastAlliance = alliance;

        return allianceChanged;
    }

    private Alliance whatisCurrentAlliance() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
            return optionalAlliance.get();
        } else {
            return null;
        }
    }

    // TODO How and when to reset (no good way to reset so once one auto or april found not able to reset unless reboot code)
    // unless disable init/end but than if in middle robot pose reset but maybe better than controls wrong way
    private void resetStateOfField() {
        m_aprilTagUsed = false;
        m_runAutoPath = false;
    }


    
}
