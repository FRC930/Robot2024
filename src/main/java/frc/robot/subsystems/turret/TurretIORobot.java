package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.IOs.TalonTurretIO;
import frc.robot.utilities.Phoenix6Utility;

/**
 * <h3>TurretIORobot</h3>
 * An IO to the real robot's turret
 */
public class TurretIORobot implements TalonTurretIO{

    protected TalonFX m_motor; // Protected because needed by IOSim
    private DutyCycleEncoder m_encoder;
    private VoltageOut m_outputRequest;

    public TurretIORobot(int motorID, int encoderID, String canbus, double gearRatio, double degreesOffset) {
        m_motor = new TalonFX(motorID, canbus);

        m_outputRequest = new VoltageOut(0.0);

        m_encoder = new DutyCycleEncoder(encoderID);

        m_encoder.setPositionOffset(Units.degreesToRotations(degreesOffset)); // We set offset in degrees, while encoder takes rotations

        Phoenix6Utility.resetTalonFxFactoryDefaults(m_motor);

        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        Phoenix6Utility.applyConfigAndNoRetry(m_motor, 
            () -> m_motor.setControl(m_outputRequest.withOutput(volts)));
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
        return Units.rotationsToDegrees(getMechRotations());
    }

    @Override
    public double getMechRotations() {
        double position = m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset();
        if (position < 0.0) { // Change negative values to wrap around back to 0.0-1.0
            position += 1.0;
        }
        return position;
    }
    
}
