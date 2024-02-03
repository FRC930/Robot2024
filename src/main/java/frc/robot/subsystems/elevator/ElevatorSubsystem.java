package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;

/**
 * <h3>ElevatorSubsystem</h3>
 * A subsystem that repr
 */
public class ElevatorSubsystem extends SubsystemBase {
    private TalonPosIO m_io; // IO for real or simulated robot
    private String m_elevatorName; // Hashcode to differentiate between the two elevatorss in debugging

    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(TalonPosIO io) {
        m_io = io;
        m_elevatorName = "" + this.hashCode();
    } 

    /**
     * <h3>setTargetHeight</h3>
     * Sets the target height of the elevator
     * @param targetHeight The height the robot will try to move to
     */
    public void setTargetHeight(double targetHeight) {
        m_io.setTarget(targetHeight);
    }

    /**
     * <h3>getHeight</h3>
     * Gets the current height of the elevator
     * @return The height of the elevator
     */
    public double getHeight() {
        return m_io.getPos();
    }

    /**
     * <h3>getTargetHeight</h3>
     * Gets the height the elevator is trying to move to.
     * @return The target height
     */
    public double getTargetHeight() {
        return m_io.getTarget();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the current velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getVelocity() {
        return m_io.getVelocity();
    }

    @Override
    public void periodic() {
        m_io.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + m_elevatorName + "/Velocity", getVelocity());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + m_elevatorName + "/Height", getHeight());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + m_elevatorName + "/SetPoint", m_io.getTarget());
        Logger.recordOutput(this.getClass().getSimpleName() + "/" + m_elevatorName + "/Voltage", m_io.getVoltage());
    }
}
