package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SetTurretPositionCommand extends Command {
    
    private static final double TURRET_SPEED = 0.3; // Run speed
    private static final double TURRET_DEADBAND = 5; // Will end command if turret pos is within this many degrees of target pos
    private double m_target;
    private TurretSubsystem m_turret;

    public SetTurretPositionCommand(TurretSubsystem turret, double degrees) {
        m_target = degrees;
        m_turret = turret;
    }

    @Override
    public void execute() {
        
        double diff = m_target - m_turret.getPosition();

        if (diff < -TURRET_DEADBAND) {
            m_turret.setSpeed(-TURRET_SPEED);
        } else if (diff > TURRET_DEADBAND) {
            m_turret.setSpeed(TURRET_SPEED);
        }

    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_target - m_turret.getPosition()) < TURRET_DEADBAND*2;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setSpeed(0.0);
    }

    
}
