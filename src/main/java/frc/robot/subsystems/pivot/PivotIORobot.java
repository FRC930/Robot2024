package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.IOs.TalonPosIO;


/**
 * <h3>PivotIORobot</h3>
 * An IO to the real robot's pivot
 */
public class PivotIORobot implements TalonPosIO{
    protected TalonFX m_motor;

    private MotionMagicExpoVoltage m_request;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     */
    public PivotIORobot(int id, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        m_motor = new TalonFX(id, canbus);

        m_request = new MotionMagicExpoVoltage(0).withEnableFOC(true);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Arm_Cosine)); // PID/FF configs
        cfg.withMotionMagic(mmConfigs); // Motion magic configs
        cfg.Feedback.RotorToSensorRatio = gearRatio; // Applies gear ratio
        
        m_motor.getConfigurator().apply(cfg); // Applies these configs ^
        m_motor.setNeutralMode(NeutralModeValue.Brake); // Enables brake mode
        

        m_motor.setControl(m_request.withPosition(0).withSlot(0)); // 
    }
    
    @Override
    public void runSim() {}

    @Override
    public double getPos() {
        return m_motor.getPosition().getValue(); 
    }

    @Override
    public double getVelocity() {
       return m_motor.getVelocity().getValue();
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
