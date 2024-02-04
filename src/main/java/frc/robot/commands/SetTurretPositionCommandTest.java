package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * <h3>SetTurretPositionCommand</h3>
 * Sets the turrets position.
 */
public class SetTurretPositionCommandTest extends SetTurretPositionCommand {

    
    /**
    * <h3>SetTurretPositionCommand</h3>
    * Constructs a command to set the turrets position.
    */
    @Deprecated
    public SetTurretPositionCommandTest(TurretSubsystem turretSubsystem, double turretPosition) {
        super(turretSubsystem, turretPosition);
        SmartDashboard.putNumber("TurretSetPosition", 0.0);
    }

    @Override
    public void initialize() {
        m_turret.setPosition(SmartDashboard.getNumber("TurretSetPosition", 0.0));
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