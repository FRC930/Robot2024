package frc.robot.subsystems.mm_turret;


import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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

    //TODO get these values from Robot Container
    private final double Velocity = RobotContainer.TURRET_REFINE_COMMAND_VELOCITY;
    private final double Acceleration = RobotContainer.TURRET_REFINE_COMMAND_ACCELERATION;
    private final double Jerk = RobotContainer.TURRET_REFINE_COMMAND_JERK;
    private final boolean EnableFOC = RobotContainer.TURRET_REFINE_COMMAND_ENABLEFOC;
    private final double FeedForward = RobotContainer.TURRET_REFINE_COMMAND_FEED_FORWARD;
    private final int Slot = RobotContainer.TURRET_REFINE_COMMAND_SLOT;
    private final boolean OverrideBrakeDurNeutral = RobotContainer.TURRET_REFINE_COMMAND_OVERRIDE_BRAKE_DUR_NEUTRAL;
    private final boolean LimitForwardMotion = RobotContainer.TURRET_REFINE_COMMAND_LIMIT_FORWARD_MOTION;
    private final boolean LimitReverseMotion = RobotContainer.TURRET_REFINE_COMMAND_LIMIT_REVERSE_MOTION;
    private DynamicMotionMagicVoltage pullConfig = new DynamicMotionMagicVoltage(0.0, Velocity, Acceleration, Jerk, EnableFOC, FeedForward, Slot, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion);
    
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
        
        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true; 
        // cfg.CurrentLimits.SupplyCurrentThreshold = 0; // the peak supply current, in amps 
        // cfg.CurrentLimits.SupplyTimeThreshold = 1.5; // the time at the peak supply current before the limit triggers, in sec
        
        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake); // Enables brake mode
        m_motor.setInverted(false); //with metal gears it was not inverted 

        // Zeros motor encoder using through bore
        m_encoder.setPositionOffset(Units.degreesToRotations(encoderOffset));

        // delayedConfigure() will run the configuration in disableInit() (delaying so bore encoder can has time to start up)
    }

    @Override
    public void delayedConfigure() {
        // TODO WHAT IF ENCODER DISCONNECTED OR BROKEN (INFINITE LOOP!!!!)
        // DISABLE TURRET given it could tear apart wiring for turret.
        try {
            while(!m_encoder.isConnected()) {
                Thread.sleep(10);
                System.out.println("****************************DUTY CYCLE ENCODER NOT RUNNING******************************************************************************");
            } 
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        double position = getAbsoluteEncoderPosition();
        
        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.getConfigurator().setPosition(position));
            
        // Move to stow pos
        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(Units.degreesToRotations(CommandFactoryUtility.TURRET_STOW_POS)).withSlot(0)));
    }

    /**
     * 
     * @return number of rotations in the range of -.5 to .5
     */
    private double getAbsoluteEncoderPosition() {
        double absolutePos = m_encoder.getAbsolutePosition();
        double offset = m_encoder.getPositionOffset();

        Logger.recordOutput("mmTurretSubsystem/SeededAbsoluteEncoderAngle",
                Units.rotationsToDegrees(absolutePos-offset));

        // Normalize value to -0.5 to 0.5  (Negative half rotation/Position half rotation)
        double position = Math.IEEEremainder(absolutePos - offset, 1.0);
        return position;
    }
    
    @Override
    public void runSim() {}

    @Override
    public double getPos() {
        getAbsoluteEncoderPosition(); // Don't use absolute encoder position, but want to log it.
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
        double position = Phoenix6Utility.getPositionFromController(m_motor, 0.0);        
        return Units.rotationsToDegrees(position);
    }

    @Override
    public void setRefinedTarget(double position) {
        Phoenix6Utility.applyConfigAndNoRetry(m_motor,
        () -> m_motor.setControl(pullConfig.withPosition(Units.degreesToRotations(position)).withSlot(0)));
    }

}
