package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.Phoenix6Utility;

/**
 * <h3>ElevatorIORobot</h3>
 * An IO to the real robot's elevator
 */
public class ElevatorIORobot implements TalonPosIO {
    protected final TalonFX leftElevatorFollower;
    protected final TalonFX rightElevatorMaster;
    protected final double gearRatio;
    private final double maxHeight;

    private final MotionMagicExpoVoltage m_request;


    private final double Position = 0;
    private final double Velocity = 0;
    private final double Acceleration = 0;
    private final double Jerk = 0;
    private final boolean EnableFOC = false;
    private final double FeedForward = 0;
    private final int Slot = 0;
    private final boolean OverrideBrakeDurNeutral = false;
    private final boolean LimitForwardMotion = false;
    private final boolean LimitReverseMotion = false;
    private DynamicMotionMagicVoltage pullConfig = new DynamicMotionMagicVoltage(Position, Velocity, Acceleration, Jerk, EnableFOC, FeedForward, Slot, OverrideBrakeDurNeutral, LimitForwardMotion, LimitReverseMotion);

    /**
     * <h3>ElevatorIORobot</h3> 
     * Creates a subsystem that represents an Elevator system
     * @param motorID The id of the elevator motor
     * @param config The PID and Feedforward controller configs
     * @param gearRatio The ratio of the motor rotations to the height on the elevator
     */
    public ElevatorIORobot (int motor1ID, int motor2ID, String canbus, Slot0Configs config, MotionMagicConfigs mmConfigs,ElevatorType elevator){
        leftElevatorFollower = new TalonFX(motor2ID, canbus);
        rightElevatorMaster = new TalonFX(motor1ID, canbus);
        this.maxHeight = elevator.m_maxHeight;
        this.gearRatio = elevator.m_gearRatio;
       
        m_request = new MotionMagicExpoVoltage(0).withEnableFOC(true);
        

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Elevator_Static));
        cfg.Feedback.SensorToMechanismRatio = this.gearRatio; //The ratio between the motor turning and the elevator moving. We may have to invert this
        cfg.withMotionMagic(mmConfigs); // The Motion Magic configs

        // Phoenix6Utility.setTalonFxConfiguration(leftElevatorFollower, cfg);
        Phoenix6Utility.setTalonFxConfiguration(rightElevatorMaster, cfg);
      //  leftElevatorFollower.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMaster.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMaster.setInverted(false);

    //    Phoenix6Utility.applyConfigAndRetry(leftElevatorFollower, 
    //         () -> leftElevatorFollower.setControl(new Follower(rightElevatorMaster.getDeviceID(), true)));
        Phoenix6Utility.applyConfigAndRetry(rightElevatorMaster, 
            () -> rightElevatorMaster.setControl(m_request.withPosition(0).withSlot(0)));
    }
    

    @Override
    public void runSim() {}

    /**
     * @param Height
     */
    @Override
    public void setTarget(double height) {
        Phoenix6Utility.applyConfigAndRetry(rightElevatorMaster, 
            () -> rightElevatorMaster.setControl(m_request.withPosition(MathUtil.clamp((height),0,maxHeight)).withSlot(0)));
    }

    /**
     * @param Height
     */
    @Override
    public void setPull(double height) {
        Phoenix6Utility.applyConfigAndRetry(rightElevatorMaster, 
            () -> rightElevatorMaster.setControl(pullConfig.withPosition(MathUtil.clamp((height),0,maxHeight))));
    }

    @Override
    public double getVelocity() {
        return rightElevatorMaster.getVelocity().getValue();
    }

    @Override
    public double getPos() {
        return rightElevatorMaster.getPosition().getValue();
    }

    @Override
    public double getVoltage() {
        return rightElevatorMaster.getMotorVoltage().getValue();
    }

    @Override
    public double getTarget() {
        return ((MotionMagicExpoVoltage) rightElevatorMaster.getAppliedControl()).Position;
    }


    @Override
    public void configure() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configure'");
    }  
}

