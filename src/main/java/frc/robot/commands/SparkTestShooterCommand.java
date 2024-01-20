package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkMaxShooterSubsystem;

public class SparkTestShooterCommand extends Command {

    private SparkMaxShooterSubsystem m_shooter;

    private double m_leftMotorSpeed;
    private double m_rightMotorSpeed;

    public SparkTestShooterCommand(SparkMaxShooterSubsystem shooter) {
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_leftMotorSpeed = MathUtil.clamp(SmartDashboard.getNumber("LeftSparkMotor", 0.0)/100, -1.0, 1.0); //Converts the inputted percentage
        m_rightMotorSpeed = MathUtil.clamp(SmartDashboard.getNumber("RightSparkMotor", 0.0)/100, -1.0, 1.0); //Converts the inputted percentage

        m_shooter.setMotorSpeed(m_leftMotorSpeed, m_rightMotorSpeed);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("LeftSparkMotorSpeed", m_shooter.gettopMotorSpeed());
        SmartDashboard.putNumber("RightSparkMotorSpeed", m_shooter.getbottomMotorSpeed());

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
