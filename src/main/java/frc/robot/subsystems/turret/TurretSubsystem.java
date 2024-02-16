package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonTurretIO;
/**
 * <h3>TurretSubsystem</h3>
 * A subsystem that represents the turret on the robot.
 */
public class TurretSubsystem extends SubsystemBase{

    private static final double VIEW_CHANGE = 180.0;
    private static final double TURRET_MIN_POS = -60.0;
    private static final double TURRET_MAX_POS = 30.0;
    public static final double STOW_POS = -45.0;

    private final TalonTurretIO m_io;

    private final ProfiledPIDController m_pid;

    private final SimpleMotorFeedforward m_ff;

    private double m_target = 0.0;
    private boolean m_isPosSet = false; // Safety check so turret doesn't try to move to 0 before a value is set


    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public TurretSubsystem(TalonTurretIO io, ProfiledPIDController pid, SimpleMotorFeedforward ff) {
        m_io = io;
        m_pid = pid;
        m_ff = ff;
    }

    /**
     * <h3>setPosition</h3>
     * Sets target position, with applied deadbands to avoid wrapping, and with offset
     * @param position Desired position on [-180, 180], with 0 being straight forward/stow
     */
    public void setPosition(double position) {
        m_target = MathUtil.clamp(position, TURRET_MIN_POS, TURRET_MAX_POS);
        m_isPosSet = true;
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from 0
     */
    public double getPosition() {
        return m_io.getDegrees() - VIEW_CHANGE;
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the velocity in degrees per second where 0 is the horizontal and positive is up.
     * @return
     */
    public double getVelocity() {
        return m_io.getSpeed();
    }

    public double getVoltage() {
        return m_io.getVoltage();
    }

    public void setVoltage(double volts) {
        if (getPosition() <= TURRET_MIN_POS && volts < 0) {
        volts = 0;
        } else if (getPosition() >= TURRET_MAX_POS && volts > 0) { // TODO: TEST AND SWITCH EFFORTS POS/NEG IF SOFT LIMITS NOT WORKING
        volts = 0;
        }
        m_io.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public double getTarget() {
        return m_target;
    }

    @Override
    public void periodic() {
        if (m_isPosSet) { // Only run if position has been set already
            double currentDegrees = getPosition();

            // Set up PID controller
            double effort = m_pid.calculate(currentDegrees, m_target);
            
            //Set up Feed Forward
            double feedforward = m_ff.calculate(Units.degreesToRadians(currentDegrees), Units.degreesToRadians(m_io.getSpeed()));

            effort += feedforward;

            m_io.setVoltage(effort);
        }

        m_io.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/TargetDegrees", getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage", getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity",getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Degrees",getPosition());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Rotations",m_io.getMechRotations());
    }
}
