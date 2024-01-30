package frc.robot.subsystems.roller;

import com.ctre.phoenix6.hardware.TalonFX;

public class RollerMotorIOReal implements RollerMotorIO {

    protected TalonFX m_motor;

    public RollerMotorIOReal(int id) {
        m_motor = new TalonFX(id);
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