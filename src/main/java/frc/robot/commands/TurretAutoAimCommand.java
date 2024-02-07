package frc.robot.commands;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.RobotOdometryUtility;

public class TurretAutoAimCommand extends Command{
    private TurretSubsystem m_TurretSubsystem;
    private Pose2d m_BlueTargetPose;
    private Pose2d m_RedTargetPose;
    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private double m_CurrentRobotHeading;
    private double m_DesiredHeading;

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y


    public TurretAutoAimCommand(TurretSubsystem turretSubsystem, Pose2d redTargetPose, Pose2d blueTargetPose) {
        m_RedTargetPose = redTargetPose;
        m_BlueTargetPose = blueTargetPose;
        m_TargetPose = blueTargetPose;

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);
    }

    @Override
    public void execute() {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
        Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = m_RedTargetPose;
            } else {
                m_TargetPose = m_BlueTargetPose;
            }
        }
        
        m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        m_CurrentRobotHeading = RobotOdometryUtility.getInstance().getRobotOdometry().getRotation().getDegrees();

        SmartDashboard.putNumber("AutoAim/RobotHeading", m_CurrentRobotHeading);

        tx = m_TargetPose.getX();
        ty = m_TargetPose.getY();
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();

        SmartDashboard.putNumber("AutoAim/tx", tx);
        SmartDashboard.putNumber("AutoAim/ty", ty);
        SmartDashboard.putNumber("AutoAim/rx", rx);
        SmartDashboard.putNumber("AutoAim/ry", ry);

        
        m_DesiredHeading = Math.toDegrees(Math.atan2(ty - ry, tx - rx)) - m_CurrentRobotHeading;

        SmartDashboard.putNumber("AutoAim/DesiredHeading", m_DesiredHeading);

        m_TurretSubsystem.setPosition(m_DesiredHeading);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
