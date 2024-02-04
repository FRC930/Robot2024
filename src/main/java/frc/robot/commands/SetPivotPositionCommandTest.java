package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotSubsystem;

/**
 * <h3>SetPivotPositionCommand</h3>
 * Sets the pivot's position
 */
@Deprecated
public class SetPivotPositionCommandTest extends SetPivotPositionCommand {


    /**
    * <h3>SetPivotPositionCommand</h3>
    * Constructs a command to set the pivot's position
    */
    public SetPivotPositionCommandTest(PivotSubsystem turretSubsystem, double turretPosition) {
        super(turretSubsystem, turretPosition);
        SmartDashboard.putNumber("PivotSetPosition", 0.0);
    }

    @Override
    public void initialize() {
        m_pivot.setPosition(SmartDashboard.getNumber("PivotSetPosition", 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_pivot.setPosition(0);
    }
        
    

}