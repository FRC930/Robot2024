package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooterCommand extends Command {

    private ShooterSubsystem m_shooter;

    private double m_leftMotorSpeed;
    private double m_rightMotorSpeed;

    public TestShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_leftMotorSpeed = MathUtil.clamp(SmartDashboard.getNumber("KrakenLeftMotor", 0.0)/100, -1.0, 1.0); //Converts the inputted percentage
        m_rightMotorSpeed = MathUtil.clamp(SmartDashboard.getNumber("KrakenRightMotor", 0.0)/100, -1.0, 1.0);

        m_shooter.setMotorSpeed(m_leftMotorSpeed, m_rightMotorSpeed);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("KrakenLeftMotorSpeed", m_shooter.getLeftMotorSpeed());
        SmartDashboard.putNumber("KrakenRightMotorSpeed", m_shooter.getRightMotorSpeed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }
}
