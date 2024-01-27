package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexedShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    
    private IntakeSubsystem m_intake;
    private double m_intakeSpeed;
    private IndexedShooterSubsystem m_shooter;
    private double m_indexSpeed;

    public RunIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed, IndexedShooterSubsystem shooterSubsystem, double indexSpeed) {
        m_intake = intakeSubsystem;
        m_intakeSpeed = intakeSpeed;
        m_shooter = shooterSubsystem;
        m_indexSpeed = indexSpeed;
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
        m_shooter.setIndexSpeed(m_indexSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getSensor();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_shooter.setIndexSpeed(0.0);
    }
}
