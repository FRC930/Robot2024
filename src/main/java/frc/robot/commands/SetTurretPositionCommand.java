package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;

/**
 * <h3>SetTurretPositionCommand</h3>
 * Sets the turrets position.
 */
public class SetTurretPositionCommand extends Command {

    protected mmTurretSubsystem m_turret;
    private double m_turretPos;
    
    /**
    * <h3>SetTurretPositionCommand</h3>
    * Constructs a command to set the turrets position.
    */
    public SetTurretPositionCommand(mmTurretSubsystem turretSubsystem, double turretPosition) {
        m_turret = turretSubsystem;
        m_turretPos = turretPosition;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        m_turret.setPosition(m_turretPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPosition(0);
    }

}