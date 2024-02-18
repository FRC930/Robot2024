package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.subsystems.mm_turret.mmTurretSubsystem;

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
    public SetTurretPositionCommandTest(mmTurretSubsystem turretSubsystem, double turretPosition) {
        super(turretSubsystem, turretPosition);
        SmartDashboard.putNumber("TurretSetPosition", 0.0);
    }

    @Override
    public void initialize() {
        m_turret.setPosition(SmartDashboard.getNumber("TurretSetPosition", 0.0));
    }
    
    @Override
    public void end(boolean interrupted) {
        /// Dont want turret to return back to 0 after setting position even after releasing button during testing phase
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}