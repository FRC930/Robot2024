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
import frc.robot.RobotContainer;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.CommandFactoryUtility;
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


    private final double Position = CommandFactoryUtility.ELEVATOR_STOW_POS;
    private final double Velocity = RobotContainer.ENDGAME_ELEVATOR_VELOCITY;
    private final double Acceleration = RobotContainer.ENDGAME_ELEVATOR_ACCELERATION;
    private final double Jerk = RobotContainer.ENDGAME_ELEVATOR_JERK;
    private final boolean EnableFOC = RobotContainer.ENDGAME_ELEVATOR_ENABLEFOC;
    private final double FeedForward = RobotContainer.ENDGAME_ELEVATOR_FEEDFORWARD;
    private final int Slot = RobotContainer.ENDGAME_ELEVATOR_SLOT;
    private final boolean OverrideBrakeDurNeutral = RobotContainer.ENDGAME_ELEVATOR_OVERRIDEBRAKEDURNEUTRAL;
    private final boolean LimitForwardMotion = RobotContainer.ENDGAME_ELEVATOR_LIMITFORWARDMOTION;
    private final boolean LimitReverseMotion = RobotContainer.ENDGAME_ELEVATOR_LIMITREVERSEMOTION;
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
        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true; 
        // cfg.CurrentLimits.SupplyCurrentThreshold = 0; // the peak supply current, in amps 
        // cfg.CurrentLimits.SupplyTimeThreshold = 1.5; // the time at the peak supply current before the limit triggers, in sec
        cfg.CurrentLimits.StatorCurrentLimit = 150.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
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
    public void setRefinedTarget(double height) {
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
        double position = Phoenix6Utility.getPositionFromController(rightElevatorMaster, 0.0);        
        return position;
    }


    @Override
    public void delayedConfigure() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configure'");
    }  
}

