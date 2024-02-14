package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetPivotPositionCommand;
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
    public void end(boolean interrupted) {
        /// Dont want pivot to return back to 0 after setting position even after releasing button during testing phase
    }
}