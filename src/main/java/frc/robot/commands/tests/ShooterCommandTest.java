package frc.robot.commands.tests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * <h3>ShooterCommand</h3>
 * Sets the speed of the two shooter motors in percent output until interrupted
 */
@Deprecated
public class ShooterCommandTest extends ShooterCommand {

    /**
    * <h3>ShooterCommand</h3>
    * Creates an instance of a shooter command
    * @param shooter    instance of shooter subsystem
    * @param leftSpeed  speed of left two motors
    * @param rightSpeed speed of right two rollers 
    */
    public ShooterCommandTest(ShooterSubsystem shooter, double leftSpeed, double rightSpeed) {
        super(shooter, leftSpeed, rightSpeed);
        SmartDashboard.putNumber("ShooterLeftMotor", 0.0);
        SmartDashboard.putNumber("ShooterRightMotor", 0.0);
    }

    @Override
    public void initialize() {
        double lSpeed = MathUtil.clamp(SmartDashboard.getNumber("ShooterLeftMotor",0.0)/100, -1.0, 1.0); //Converts the inputted percentage
        double rSpeed = MathUtil.clamp(SmartDashboard.getNumber("ShooterRightMotor", 0.0)/100, -1.0, 1.0);

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