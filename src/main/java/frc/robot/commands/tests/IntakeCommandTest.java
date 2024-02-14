package frc.robot.commands.tests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.IntakeCommand;
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
        m_intake.setSpeed(MathUtil.clamp(SmartDashboard.getNumber("IntakeMotor", 0.0)/100,-1,1));
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
