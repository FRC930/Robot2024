package frc.robot.commands;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.RobotOdometryUtility;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Automatically aims the turret to one of the speakers based on the alliance.
public class TurretAutoAimCommand extends Command{
    private TurretSubsystem m_TurretSubsystem;
    private Pose2d m_BlueTargetPose;
    private Pose2d m_RedTargetPose;
    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private double m_CurrentRobotHeading;
    private double m_DesiredHeading;

    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y
    
    /** Aims the turret to the speaker apriltags based on the current alliance.
     * Constructor
     * @param turretSubsystem the subsystem that controls the turret.
     */
    public TurretAutoAimCommand(TurretSubsystem turretSubsystem) {
        m_RedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        m_BlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        m_TargetPose = m_BlueTargetPose;

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);
    }
    
    @Override
    public void execute() {
       // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = m_RedTargetPose;
            } else {
                m_TargetPose = m_BlueTargetPose;
            }
        }
        // gets the robots position, and gets the robots heading.
        m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        m_CurrentRobotHeading = m_CurrentPose.getRotation().getDegrees();
        // logs the robot heding
        SmartDashboard.putNumber("AutoAim/RobotHeading", m_CurrentRobotHeading);
        
        // sets the target x/y, and sets the robots x/y
        tx = m_TargetPose.getX();
        ty = m_TargetPose.getY();
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();
        
        //Logs the values above.
        SmartDashboard.putNumber("AutoAim/tx", tx);
        SmartDashboard.putNumber("AutoAim/ty", ty);
        SmartDashboard.putNumber("AutoAim/rx", rx);
        SmartDashboard.putNumber("AutoAim/ry", ry);

        // calculates how far we need to rotate the turret to get to the desired position based on:
        // robots turret heading - the robots base heading
        m_DesiredHeading = -Math.IEEEremainder(Math.toDegrees(Math.atan2(ty - ry, tx - rx)) - m_CurrentRobotHeading, 360);

        //Logs the desired heading
        SmartDashboard.putNumber("AutoAim/Math", Math.toDegrees(Math.atan2(ty - ry, tx - rx)));
        SmartDashboard.putNumber("AutoAim/DesiredHeading", m_DesiredHeading);

        // actually moves the robots turret to the desired position
        // TODO sussex back in
        //m_TurretSubsystem.setTarget(m_DesiredHeading);
    }

    // Makes it so that this command never ends.
    @Override
    public boolean isFinished() {
        return false;
    }
}
