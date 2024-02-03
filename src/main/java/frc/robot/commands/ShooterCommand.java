package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private ShooterSubsystem m_shooter;

    private double m_leftMotorSpeed;
    private double m_rightMotorSpeed;

    public ShooterCommand(ShooterSubsystem shooter, double leftSpeed, double rightSpeed) {
        m_shooter = shooter;
        m_leftMotorSpeed = leftSpeed;
        m_rightMotorSpeed = rightSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double lSpeed = MathUtil.clamp(m_leftMotorSpeed, -1.0, 1.0); //Converts the inputted percentage
        double rSpeed = MathUtil.clamp(m_rightMotorSpeed, -1.0, 1.0);

        m_shooter.setMotorSpeed(lSpeed, rSpeed);
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
