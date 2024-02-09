package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * <h3>ShooterCommand</h3>
 * Sets the speed of the two shooter motors in percent output until interrupted
 */
public class ShooterCommand extends Command {

    protected ShooterSubsystem m_shooter;

    private double m_leftMotorSpeed;
    private double m_rightMotorSpeed;

    /**
    * <h3>ShooterCommand</h3>
    * Creates an instance of a shooter command
    * @param shooter    instance of shooter subsystem
    * @param leftSpeed  speed of left two motors
    * @param rightSpeed speed of right two rollers 
    */
    public ShooterCommand(ShooterSubsystem shooter, double leftSpeed, double rightSpeed) {
        m_shooter = shooter;
        m_leftMotorSpeed = leftSpeed;
        m_rightMotorSpeed = rightSpeed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double lSpeed = MathUtil.clamp(m_leftMotorSpeed, -1.0, 1.0); //Converts the inputted percentage
        double rSpeed = MathUtil.clamp(m_rightMotorSpeed, -1.0, 1.0); //TODO remove clamp

        m_shooter.setSpeed(lSpeed, rSpeed,1,1);
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
