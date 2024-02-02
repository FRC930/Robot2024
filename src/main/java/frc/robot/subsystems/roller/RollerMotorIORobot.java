package frc.robot.subsystems.roller;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.IOs.TalonRollerIO;

/**
 * <h3>RollerMotorIORobot</h3>
 * Representation of a roller motor on the robot
 */
public class RollerMotorIORobot implements TalonRollerIO {

    protected TalonFX m_motor; // Protected because needed by IOSim

    public RollerMotorIORobot(int id, String canbus) {
        m_motor = new TalonFX(id, canbus);
    }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public double getSpeed() {
        return m_motor.getVelocity().getValue();
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public void runSim() {}

    @Override
    public TalonFX getTalon() {
        return m_motor;
    }
}