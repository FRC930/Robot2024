package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.SpeakerScoreUtility;

/**
 * <h3>PivotSubsystem</h3>
 * A subsystem that represents the pivot
 */
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
        m_io.setTarget(MathUtil.clamp(angle,0.0,90.0));
        
    }

    /**
     * <h3>getSetPoint</h3>
     * Gets the angle that the subsystem is currently trying to turn to
     * @param angle The angle in degrees from the horizontal
     */
    public double getTarget() {
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
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + pivotName + "/Velocity", getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + pivotName + "/Angle", getPosition());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + pivotName + "/SetPoint", getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + pivotName + "/Voltage", m_io.getVoltage());
        
    }

    public Command newSetPosCommand(double pos) {
        return new InstantCommand(() -> setPosition(pos), this);
    }

    public Command newSetPosCommand(SpeakerScoreUtility speakerUtil) {
        return new InstantCommand(() -> setPosition(speakerUtil.getPivotAngle()), this);
    }

    public boolean atSetpoint(double deadband) {
        double pos = getPosition();
        double target = getTarget();
        return MathUtil.applyDeadband(target - pos, deadband) == 0.0;
    }

    public Command newWaitUntilSetpointCommand(double seconds, double deadband) {
        return new WaitCommand(seconds).until(() -> atSetpoint(deadband)); // Not dependent on subsystem because can run parralel with set position
    }
}
