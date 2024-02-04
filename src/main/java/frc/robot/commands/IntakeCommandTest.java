package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * <h3>IntakeCommand</h3>
 * Sets the intake speed
 */
@Deprecated
public class IntakeCommandTest extends IntakeCommand {
    
    /**
    * <h3>IntakeCommand</h3>
    * Constructs a command to set the intake speed.
    */
    public IntakeCommandTest(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        super(intakeSubsystem, intakeSpeed);
        SmartDashboard.putNumber("IntakeMotor", 0.0);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(MathUtil.clamp(SmartDashboard.getNumber("IntakeMotor", 0.0)/100,-1,1));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
