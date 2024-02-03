package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    
    private IntakeSubsystem m_intake;
    private double m_intakeSpeed;
    public RunIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        m_intake = intakeSubsystem;
        m_intakeSpeed = intakeSpeed; // Add intake
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
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
