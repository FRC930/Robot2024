package frc.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.sim.PhysicsSim;

public class TurretIORobot implements TurretIO{
    private TalonFX m_motor;
    private MotionMagicVoltage m_request;

    public TurretIORobot(TalonFX motor, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        m_motor = motor;

        m_request = new MotionMagicVoltage(0).withEnableFOC(true);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config);
        cfg.withMotionMagic(mmConfigs);
        cfg.Feedback.RotorToSensorRatio = gearRatio;
        
        m_motor.getConfigurator().apply(cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake);

        m_motor.setControl(m_request.withPosition(0).withSlot(0));

        //TODO: TEMP
        if(Utils.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(m_motor, 0.001);
        }
    }
    @Override
    public void updateInputs() {}

    @Override
    public double getCurrentAngleDegrees() {
        return m_motor.getPosition().getValue(); // TODO: make sure this is right direction
    }

    @Override
    public double getVelocityDegreesPerSecond() {
       return m_motor.getVelocity().getValue(); // TODO: make sure this is right direction
    }

    @Override
    public void setPosition(double position) {
        m_motor.setControl(m_request.withPosition(position).withSlot(0));
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public double getSetPoint() {
        return ((MotionMagicVoltage)m_motor.getAppliedControl()).Position;
    }
    
}
