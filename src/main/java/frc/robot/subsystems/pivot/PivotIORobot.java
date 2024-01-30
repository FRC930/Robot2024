package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.PosSubsystemIO;


public class PivotIORobot implements PosSubsystemIO{
    protected TalonFX m_motor;

    private MotionMagicExpoVoltage m_request;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     * @param motorID The id of the pivot motor
     */
    public PivotIORobot(TalonFX motor, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        m_motor = motor;

        m_request = new MotionMagicExpoVoltage(0).withEnableFOC(true);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config);
        cfg.withMotionMagic(mmConfigs);
        cfg.Feedback.RotorToSensorRatio = gearRatio;
        
        m_motor.getConfigurator().apply(cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        

        m_motor.setControl(m_request.withPosition(0).withSlot(0));
    }
    
    @Override
    public void runSim() {}

    @Override
    public double getPos() {
        return m_motor.getPosition().getValue(); // TODO: make sure this is right direction
    }

    @Override
    public double getVelocity() {
       return m_motor.getVelocity().getValue(); // TODO: make sure this is right direction
    }

    @Override
    public void setTarget(double position) {
        m_motor.setControl(m_request.withPosition(position).withSlot(0));
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public double getTarget() {
        return ((MotionMagicExpoVoltage)m_motor.getAppliedControl()).Position;
    }
}
