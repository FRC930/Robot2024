package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;
import frc.robot.utilities.LimeLightDetectionUtility;

public class TurretRefineCommand extends Command{

    private LimeLightDetectionUtility m_LimeLightDetectionUtility = new LimeLightDetectionUtility("limelight-turret");
    
    private mmTurretSubsystem m_TurretSubsystem;
    private double m_TurretPosition;
    private double m_AprilTagAngle;
    private double m_DeadBand = 1.0;

    public TurretRefineCommand(mmTurretSubsystem turretSubsystem) {
        m_TurretSubsystem = turretSubsystem;
        addRequirements(m_TurretSubsystem);
    }

    @Override
    public void execute() {
        m_TurretPosition = m_TurretSubsystem.getPosition();
        m_AprilTagAngle = m_LimeLightDetectionUtility.get_tx();

        SmartDashboard.putNumber("TurretAim/TX", m_AprilTagAngle);
        SmartDashboard.putBoolean("TurretAim/isFinished", (Math.abs(m_AprilTagAngle) <= m_DeadBand) && (m_AprilTagAngle != 0.0));

        m_TurretSubsystem.setRefinedTarget(m_TurretPosition + m_AprilTagAngle);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_AprilTagAngle) <= m_DeadBand) && (m_AprilTagAngle != 0.0);
    }  
    

}
