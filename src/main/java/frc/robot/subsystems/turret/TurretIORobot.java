package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.IOs.TalonTurretIO;

/**
 * <h3>TurretIORobot</h3>
 * An IO to the real robot's turret
 */
public class TurretIORobot implements TalonTurretIO{

    protected TalonFX m_motor; // Protected because needed by IOSim
    private DutyCycleEncoder m_encoder;

    public TurretIORobot(int motorID, int encoderID, String canbus, double offset) {
        m_motor = new TalonFX(motorID, canbus);

        m_encoder = new DutyCycleEncoder(encoderID);

        m_encoder.setPositionOffset(offset);

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
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

    @Override
    public double getDegrees() {
        return getMechRotations() * 360; // Multiply by 360 to convert from rotations to degrees in 1:1 gear ratio 
        // TODO: Verify conversion
    }

    @Override
    public double getMechRotations() {
        return m_encoder.get();
    }
    
}