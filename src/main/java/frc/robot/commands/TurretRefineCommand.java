package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;
import frc.robot.utilities.LimeLightDetectionUtility;
import frc.robot.utilities.RobotOdometryUtility;

public class TurretRefineCommand extends Command{

    private LimeLightDetectionUtility m_LimeLightDetectionUtility = new LimeLightDetectionUtility("limelight-turret");
    
    private mmTurretSubsystem m_TurretSubsystem;
    private double m_TurretPosition;
    private double m_AprilTagAngle;
    private double m_DeadBand = 1.0;
    private Pose2d m_CurrentPose;

    private Pose2d m_RedTargetPose;
    private Pose2d m_BlueTargetPose;
    private Pose2d m_TargetPose;

    private AprilTagFieldLayout m_AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    public TurretRefineCommand(mmTurretSubsystem turretSubsystem) {
        m_RedTargetPose = m_AprilTagFieldLayout.getTagPose(4).get().toPose2d();
        m_RedTargetPose = new Pose2d(m_RedTargetPose.getX() + 0.5, m_RedTargetPose.getY(), m_RedTargetPose.getRotation());
        m_BlueTargetPose = m_AprilTagFieldLayout.getTagPose(7).get().toPose2d();
        m_BlueTargetPose = new Pose2d(m_BlueTargetPose.getX() - 0.5, m_BlueTargetPose.getY(), m_BlueTargetPose.getRotation());
        m_TargetPose = m_BlueTargetPose;

        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);
    }

    @Override
    public void execute() {
        boolean isBlue;
        // If there is an alliance present it sets the target pose based on the alliance; otherwise defaults to blue.
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isPresent()){
            Alliance alliance = optionalAlliance.get();
            if (alliance == Alliance.Red) {
                m_TargetPose = m_RedTargetPose;
                isBlue = false;
            } else {
                m_TargetPose = m_BlueTargetPose;
                isBlue = true;
            }
        } else {
            isBlue = true;
        }

        m_CurrentPose = RobotOdometryUtility.getInstance().getRobotOdometry();
        m_TurretPosition = m_TurretSubsystem.getPosition();
        m_AprilTagAngle = m_LimeLightDetectionUtility.get_tx();

        SmartDashboard.putNumber("TurretAim/TX", m_AprilTagAngle);
        SmartDashboard.putBoolean("TurretAim/isFinished", (Math.abs(m_AprilTagAngle) <= m_DeadBand) && (m_AprilTagAngle != 0.0));

        double Offset = 0.0;
        if ((Math.hypot(m_CurrentPose.getX() - m_TargetPose.getX(), m_CurrentPose.getY() - m_TargetPose.getY()) <= 4.0) &&
                ((m_CurrentPose.getY() > m_TargetPose.getY() + 0.5) ||
                (m_CurrentPose.getY() < m_TargetPose.getY() - 0.5))) {
                
            if ((isBlue && (m_CurrentPose.getY() < m_TargetPose.getY()))   || 
                (!isBlue && !(m_CurrentPose.getY() < m_TargetPose.getY())) ) {
                Offset = 5.0; //TODO figure out Offset values
            } else {
                Offset = -5.0;
            }

        }
        Logger.recordOutput("TurretRefineCommand/Offset", Offset);
        Logger.recordOutput("TurretRefineCommand/isNear", (Offset != 0.0) ? true : false);

        m_TurretSubsystem.setRefinedTarget(m_TurretPosition + m_AprilTagAngle + Offset);

    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_AprilTagAngle) <= m_DeadBand) && (m_AprilTagAngle != 0.0);
    }  
    

}
