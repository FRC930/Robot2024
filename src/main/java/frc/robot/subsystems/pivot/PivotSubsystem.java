package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;

public class PivotSubsystem extends SubsystemBase{

    private final TalonPosIO m_io;

    private final String pivotName;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public PivotSubsystem(TalonPosIO io) {
        m_io = io;
        pivotName = "" + this.hashCode();
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        m_io.setTarget(MathUtil.clamp(angle,0,180));
        
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getSetPoint() {
        return m_io.getTarget();
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return m_io.getPos();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return m_io.getVelocity();
    }

    /**
     * <h3>getVoltage</h3>
     * Gets the current voltage of the subystem.
     * @return The voltage the motor is running at.
     */
   // public double getVoltage() {

   // }

    @Override
    public void periodic() {
        m_io.runSim();
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Height", getPosition());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/SetPoint", getSetPoint());
        SmartDashboard.putNumber("Pivot-" + pivotName + "/Voltage", m_io.getVoltage());
        
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setPosition(90); System.out.println("Pivot Test Start");}, ()->{setPosition(0);}, this);
    }

}
