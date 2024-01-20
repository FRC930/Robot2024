package frc.robot.utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

// MotorIO base interface extends MotorController
// Implementations for SparkMax & Talon (more added later if needed)
//

public class TalonWrapper extends TalonFX implements MotorIO{

    public TalonWrapper(int motorID) {
        super(motorID);
    }

    @Override
    public void resetToFactoryDefaults() {
        super.getConfigurator().apply(new TalonFXConfiguration());
    }

    @Override
    public void setVoltage(double volts) {
        super.setVoltage(MathUtil.clamp(volts, -getMaxVoltage(), getMaxVoltage()));
    }

    @Override
    public void setShouldBrake(boolean shouldBrake) {
        super.setNeutralMode(shouldBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public double getMaxVoltage() {
        return 12;
    }

    @Override
    public double getIOVelocity() {
        return Units.RadiansPerSecond.convertFrom(super.getVelocity().getValue().doubleValue(), Units.RotationsPerSecond);
    }

    @Override
    public double getIOPosition() {
        return Units.Radians.convertFrom(super.getPosition().getValue(), Units.Rotations);
    }

    public void followIO(TalonWrapper other,boolean inverted) {
        this.setControl(new Follower(other.getDeviceID(), inverted));
    }
}
