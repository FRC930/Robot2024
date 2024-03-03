package frc.robot.subsystems.mm_turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.SpeakerScoreUtility;

/**
 * <h3>PivotSubsystem</h3>
 * A subsystem that represents the pivot
 */
public class mmTurretSubsystem extends SubsystemBase{

    private final TalonPosIO m_io;

    private final String turretName;

    private static final double VIEW_CHANGE = 0.0;
    private static final double TURRET_MIN_POS = -30.0;//-160.0;//137.0
    private static final double TURRET_MAX_POS = 30.0;//110.0;//115.0
    public static final double TURRET_DEADBAND = 2.0;
    private boolean m_isTurretLocked;

    /**
     * <h3>PivotSubsystem</h3>
     * Creates a subsystem that represents the turret's vertical pivot on the robot.
     * <p>By default, angular measures are positive going up, negative going down, and 0 at the default horizontal
     * @param motorID
     */
    public mmTurretSubsystem(TalonPosIO io) {
        m_io = io;
        turretName = "" + this.hashCode();
        m_isTurretLocked = false;
    }

    /**
     * <h3>setPosition</h3>
     * Sets the target angle of the subsystem
     * @param angle The angle in degrees from the horizontal
     */
    public void setPosition(double angle) {
        double clampedPosition = MathUtil.clamp(angle,TURRET_MIN_POS,TURRET_MAX_POS) + VIEW_CHANGE;
        Logger.recordOutput(this.getClass().getSimpleName() + "/ClampedPosition", clampedPosition);
        m_io.setTarget(clampedPosition);
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getTarget() {
        return m_io.getTarget() - VIEW_CHANGE;
    }

    /**
     * <h3>getPosition</h3>
     * Gets the current angle of the subsystem.
     * @return The angle in degrees from the horizontal
     */
    public double getPosition() {
        return m_io.getPos() - VIEW_CHANGE;
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
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity", getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Angle", getPosition());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage", m_io.getVoltage());
        
    }
    
    public InstantCommand newSetPosCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos), this);
    }

    public boolean atSetpoint() {
        double pos = getPosition();
        double target = getTarget();
        return MathUtil.applyDeadband(target - pos, TURRET_DEADBAND) == 0.0;
    }

    public Command newWaitUntilSetpointCommand(double seconds) {
        return new WaitCommand(seconds).until(() -> atSetpoint()); // Not dependent on subsystem because can run parralel with set position
    }

    public void toggleTurretLock() {
        m_isTurretLocked = !m_isTurretLocked;
    }

    public void disableTurretLock() {
        m_isTurretLocked = false;
    }

    public void enableTurretLock() {
        m_isTurretLocked = true;
    }

    public boolean getTurretLock() {
        return m_isTurretLocked;
    }

    public void setRefinedTarget(double position) {
        m_io.setRefinedTarget(position);
    }
}
