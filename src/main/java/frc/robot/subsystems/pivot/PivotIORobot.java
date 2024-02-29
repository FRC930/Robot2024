package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.Phoenix6Utility;


/**
 * <h3>PivotIORobot</h3>
 * An IO to the real robot's pivot
 */
public class PivotIORobot implements TalonPosIO{
    protected TalonFX m_motor;

    private MotionMagicVoltage m_request;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     */
    public PivotIORobot(int id, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {
        m_motor = new TalonFX(id, canbus);

        m_request = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Arm_Cosine)); // PID/FF configs
        cfg.withMotionMagic(mmConfigs); // Motion magic configs
        cfg.Feedback.SensorToMechanismRatio = gearRatio; // Applies gear ratio

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true; 
        cfg.CurrentLimits.SupplyCurrentThreshold = 0; // the peak supply current, in amps 
        cfg.CurrentLimits.SupplyTimeThreshold = 1.5; // the time at the peak supply current before the limit triggers, in sec
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 150.0;

        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake); // Enables brake mode
        m_motor.setInverted(false); //with metal gears it was not inverted 

        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(0).withSlot(0)));
    }
    
    @Override
    public void runSim() {}

    @Override
    public double getPos() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValue()); 
    }

    @Override
    public double getVelocity() {
       return Units.rotationsToDegrees(m_motor.getVelocity().getValue());
    }

    @Override
    public void setTarget(double position) {
        Phoenix6Utility.applyConfigAndNoRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(Units.degreesToRotations(position)).withSlot(0)));
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public double getTarget() {
        // return Units.rotationsToDegrees(((MotionMagicVoltage)m_motor.getAppliedControl()).Position);
        double position = Phoenix6Utility.getPositionFromController(m_motor, 0.0);        
        return Units.rotationsToDegrees(position);
    }

    @Override
    public void delayedConfigure() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configure'");
    }

    @Override
    public void setRefinedTarget(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPull'");
    }
}
