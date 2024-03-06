package frc.robot.subsystems.pivot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.Phoenix6Utility;


/**
 * <h3>PivotIORobot</h3>
 * An IO to the real robot's pivot
 */
public class PivotIORobot implements TalonPosIO{
    protected TalonFX m_motor;

    private MotionMagicVoltage m_request;

    protected final CANcoder m_cc;
    
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     */
    public PivotIORobot(int id, int canCoderId, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs) {

         m_cc = new CANcoder(canCoderId, canbus);

        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        // 
        // By default, positive direction is counter-clockwise rotation of the magnet when looking at the magnet side of the CANCoder.
        // Use the self-test snapshot to confirm the sensor value changes as expected for the chosen direction.
        // https://docs.ctre-phoenix.com/en/stable/ch12a_BringUpCANCoder.html#:~:text=By%20default%2C%20positive%20direction%20is%20counter-clockwise%20rotation%20of,looking%20at%20the%20magnet%20side%20of%20the%20CANCoder.
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // NEED TO CONFIGURE
        // cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // NEED TO CONFIGURE
        // TODO Use tuner x to zero and save number?
        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
        cc_cfg.MagnetSensor.MagnetOffset = 0.4; // TODO NEED to configure

        if(!PivotSubsystem.ENABLE_REZEROING) {
            Phoenix6Utility.applyConfigAndRetry(m_cc, () -> {return m_cc.getConfigurator().apply(cc_cfg);});
        }


        m_motor = new TalonFX(id, canbus);

        m_request = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Arm_Cosine)); // PID/FF configs
        cfg.withMotionMagic(mmConfigs); // Motion magic configs

        

        if(!PivotSubsystem.ENABLE_REZEROING) {
            cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
            cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            cfg.Feedback.RotorToSensorRatio = gearRatio;  // TODO NEED to configure
            cfg.Feedback.SensorToMechanismRatio = 1; // Applies gear ratio
        } else {
            cfg.Feedback.SensorToMechanismRatio = gearRatio; // Applies gear ratio
        }

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
        if(!PivotSubsystem.ENABLE_REZEROING) {
            SmartDashboard.putNumber("pivot/cc_pos", m_cc.getPosition().getValue());
        }
        SmartDashboard.putNumber("pivot/motor_pos", m_motor.getRotorPosition().getValue());

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