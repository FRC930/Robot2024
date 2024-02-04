package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.IOs.TalonPosIO;

/**
 * <h3>ElevatorIORobot</h3>
 * An IO to the real robot's elevator
 */
public class ElevatorIORobot implements TalonPosIO {
    protected final TalonFX leftElevatorFollower;
    protected final TalonFX rightElevatorMaster;
    private final double gearRatio;
    private final double maxHeight;

    private final MotionMagicExpoVoltage m_request;

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

       // leftElevatorFollower.getConfigurator().apply(cfg);
        rightElevatorMaster.getConfigurator().apply(cfg);
      //  leftElevatorFollower.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMaster.setNeutralMode(NeutralModeValue.Brake);
        rightElevatorMaster.setInverted(true);

      //  leftElevatorFollower.setControl(new Follower(rightElevatorMaster.getDeviceID(), true));
        rightElevatorMaster.setControl(m_request.withPosition(0).withSlot(0));
    }
    

    @Override
    public void runSim() {}

    /**
     * @param Height
     */
    @Override
    public void setTarget(double height) {
        rightElevatorMaster.setControl(m_request.withPosition(MathUtil.clamp((height),0,maxHeight)).withSlot(0));
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
}

