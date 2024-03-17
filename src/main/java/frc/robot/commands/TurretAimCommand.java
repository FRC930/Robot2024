package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.RobotOdometryUtility;
import frc.robot.utilities.SpeakerScoreUtility;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Automatically aims the turret to one of the speakers based on the alliance.
public class TurretAimCommand extends Command{

    private static final double AIM_OFFSET = Units.inchesToMeters(23.0); // May be dynamic
    private static final double NON_AMP_AIM_OFFSET = Units.inchesToMeters(13.0); // May be dynamic

    private TurretSubsystem m_TurretSubsystem;
    private Pose2d m_AmpSideBlueTargetPose;
    private Pose2d m_AmpSideRedTargetPose;
    private Pose2d m_NonAmpSideBlueTargetPose;
    private Pose2d m_NonAmpSideRedTargetPose;
    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private double m_CurrentRobotHeading;
    private double m_DesiredHeading;

    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y
    private boolean ampSide;
    
    /** Aims the turret to the speaker apriltags based on the current alliance.
     * Constructor
     * @param turretSubsystem the subsystem that controls the turret.
     */
    public TurretAimCommand(TurretSubsystem turretSubsystem) {
        m_AmpSideRedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        m_AmpSideRedTargetPose = new Pose2d(m_AmpSideRedTargetPose.getX() + 0.5, m_AmpSideRedTargetPose.getY() - Units.inchesToMeters(23.0), m_AmpSideRedTargetPose.getRotation());
        m_AmpSideBlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        m_AmpSideBlueTargetPose = new Pose2d(m_AmpSideBlueTargetPose.getX() - 0.5, m_AmpSideBlueTargetPose.getY() + Units.inchesToMeters(10.0), m_AmpSideBlueTargetPose.getRotation());
        
        m_NonAmpSideRedTargetPose = new Pose2d(m_AmpSideRedTargetPose.getX(), m_AmpSideRedTargetPose.getY() + NON_AMP_AIM_OFFSET, m_AmpSideRedTargetPose.getRotation());
        m_NonAmpSideBlueTargetPose = new Pose2d(m_AmpSideBlueTargetPose.getX(), m_AmpSideBlueTargetPose.getY() + Units.inchesToMeters(30), m_AmpSideBlueTargetPose.getRotation());

        m_TargetPose = m_AmpSideBlueTargetPose;

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);
    }
    
    @Override
    public void execute() {

        // gets the robots position, and gets the robots heading.
        m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        m_CurrentRobotHeading = m_CurrentPose.getRotation().getDegrees();
        // logs the robot heding
        // SmartDashboard.putNumber("AutoAim/RobotHeading", m_CurrentRobotHeading);
        
        // sets the target x/y, and sets the robots x/y
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();

        if (ry >= 4.5 /*Below front pillar y (in meters)*/) {
            ampSide = true;
        } else {
            ampSide = false;
        }
        Logger.recordOutput("AutoAim/ampSide", ampSide);

       // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = (ampSide)?m_AmpSideRedTargetPose:m_NonAmpSideRedTargetPose;
                if (SpeakerScoreUtility.inchesToSpeaker() > Units.metersToInches(8.0)) {
                    m_TargetPose = new Pose2d(m_TargetPose.getX(), m_TargetPose.getY() - Units.inchesToMeters(70.0), m_TargetPose.getRotation());
                }
            } else {
                m_TargetPose = (ampSide)?m_AmpSideBlueTargetPose:m_NonAmpSideBlueTargetPose;
                if (SpeakerScoreUtility.inchesToSpeaker() > Units.metersToInches(8.0)) {
                    m_TargetPose = new Pose2d(m_TargetPose.getX(), m_TargetPose.getY() + 4.5, m_TargetPose.getRotation());
                }
            }
        }

        tx = m_TargetPose.getX();
        ty = m_TargetPose.getY();
        
        //Logs the values above.
        Logger.recordOutput("AutoAim/tx", tx);
        Logger.recordOutput("AutoAim/ty", ty);
        Logger.recordOutput("AutoAim/rx", rx);
        Logger.recordOutput("AutoAim/ry", ry);

        // calculates how far we need to rotate the turret to get to the desired position based on:
        // robots turret heading - the robots base heading
        m_DesiredHeading = -Math.IEEEremainder(Math.toDegrees(Math.atan2(ty - ry, tx - rx)) - m_CurrentRobotHeading, 360);

        //Logs the desired heading
        // SmartDashboard.putNumber("AutoAim/Math", Math.toDegrees(Math.atan2(ty - ry, tx - rx)));
        Logger.recordOutput("AutoAim/DesiredHeading", m_DesiredHeading);

        // actually moves the robots turret to the desired position
        // TODO sussex back in
        m_TurretSubsystem.setPosition(m_DesiredHeading);
    }

    // Makes it so that this command never ends.
    @Override
    public boolean isFinished() {
        return false;
    }
}
