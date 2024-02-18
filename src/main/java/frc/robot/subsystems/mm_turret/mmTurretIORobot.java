package frc.robot.subsystems.mm_turret;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.Phoenix6Utility;


/**
 * <h3>PivotIORobot</h3>
 * An IO to the real robot's pivot
 */
public class mmTurretIORobot implements TalonPosIO{
    protected TalonFX m_motor;

    private MotionMagicVoltage m_request;
    private DutyCycleEncoder m_encoder;
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     */
    public mmTurretIORobot(int id, int encoderID, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs, double encoderOffset) {
        m_motor = new TalonFX(id, canbus);
        m_encoder = new DutyCycleEncoder(encoderID);

        m_request = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Elevator_Static)); // PID/FF configs
        cfg.withMotionMagic(mmConfigs); // Motion magic configs
        cfg.Feedback.SensorToMechanismRatio = gearRatio; // Applies gear ratio
        
        
        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake); // Enables brake mode
        m_motor.setInverted(false); //with metal gears it was not inverted 

        // Zeros motor encoder using through bore
        m_encoder.setPositionOffset(Units.degreesToRotations(encoderOffset));

        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.getConfigurator().setPosition(m_encoder.getAbsolutePosition()));

        // Move to stow pos
        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(Units.degreesToRotations(CommandFactoryUtility.TURRET_STOW_POS)).withSlot(0)));
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
        return Units.rotationsToDegrees(((MotionMagicVoltage)m_motor.getAppliedControl()).Position);
    }
}
