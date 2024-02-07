package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utilities.RobotOdometryUtility;

public class TurretAutoAimCommand extends Command{
    private TurretSubsystem m_TurretSubsystem;
    private Pose2d m_TargetPose;
    private Pose2d m_CurrentPose;
    private double m_DesiredHeading;

    private double tx; //target x
    private double ty; //target y
    private double rx; //robot x
    private double ry; //robot y


    public TurretAutoAimCommand(TurretSubsystem turretSubsystem, Pose2d targetPose) {
        m_TurretSubsystem = turretSubsystem;
        m_TargetPose = targetPose;
        addRequirements(m_TurretSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();

        tx = m_TargetPose.getX();
        ty = m_TargetPose.getY();
        rx = m_CurrentPose.getX();
        ry = m_CurrentPose.getY();

        SmartDashboard.putNumber("AutoAim/tx", tx);
        SmartDashboard.putNumber("AutoAim/ty", ty);
        SmartDashboard.putNumber("AutoAim/rx", rx);
        SmartDashboard.putNumber("AutoAim/ry", ry);

        
        m_DesiredHeading = Math.atan2(ty - ry, tx - rx);

        SmartDashboard.putNumber("AutoAim/DesiredHeading", m_DesiredHeading);

        m_TurretSubsystem.setPosition(Math.toDegrees(m_DesiredHeading));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
