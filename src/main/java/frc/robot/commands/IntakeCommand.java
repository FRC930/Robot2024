package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * <h3>IntakeCommand</h3>
 * Sets the intake speed
 */
public class IntakeCommand extends Command {
    
    protected IntakeSubsystem m_intake;
    private double m_intakeSpeed;
    /**
    * <h3>IntakeCommand</h3>
    * Constructs a command to set the intake speed.
    */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        m_intake = intakeSubsystem;
        m_intakeSpeed = intakeSpeed; // Add intake
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intake.setSpeed(m_intakeSpeed);
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
