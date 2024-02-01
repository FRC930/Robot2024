package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonPosIO;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonPosIO m_io;
    private String m_elevatorName;

    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(TalonPosIO io) {
        this.m_io = io;
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
        SmartDashboard.putNumber("Elevator-" + m_elevatorName + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator-" + m_elevatorName + "/Height", getHeight());
        SmartDashboard.putNumber("Elevator-" + m_elevatorName + "/SetPoint", m_io.getTarget());
        SmartDashboard.putNumber("Elevator-" + m_elevatorName + "/Voltage", m_io.getVoltage());
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setTargetHeight(5);System.out.println("Elevator Test Start");},()->{setTargetHeight(0);},this);
    }
}
