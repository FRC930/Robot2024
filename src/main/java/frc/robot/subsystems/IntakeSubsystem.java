package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


/**
 * This subsystem controls the intake
 */
public class IntakeSubsystem extends SubsystemBase {

    private static final double TRIGGER_DISTANT = 200;
    private TalonFX m_leaderMotor;
    private MotionMagicVelocityVoltage m_request;
    private TalonFX m_followerMotor;
    private TimeOfFlight m_sensor;
    private DigitalInput m_sensorSim;

    private final SlotConfigs PIDFF_CONFIG = new SlotConfigs()
        //PID
        .withKP(1)
        .withKI(0)
        .withKD(0)
        //FeedForward
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(1);

    private final MotionMagicConfigs MM_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1) // Motor target acceleration
        .withMotionMagicJerk(1); // Motor max acceleration rate of change
        
    private final double GEAR_RATIO = 1;


    /**
     * 
     * This subsystem controls the intake
     * 
     * @param leaderID The ID to the leader motor
     * @param followerID The ID to the follower motor
     * @param sensorID 
     */
    public IntakeSubsystem(int leaderID, int followerID, int sensorID)  {
        m_leaderMotor = new TalonFX(leaderID);
        m_followerMotor = new TalonFX(followerID);

        m_request = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withSlot0(Slot0Configs.from(PIDFF_CONFIG));
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        config.withMotionMagic(MM_CONFIGS); // The Motion Magic configs
        

        m_leaderMotor.getConfigurator().apply(config);
        m_leaderMotor.setNeutralMode(NeutralModeValue.Coast);
        m_followerMotor.getConfigurator().apply(config);
        m_followerMotor.setNeutralMode(NeutralModeValue.Coast);

        m_followerMotor.setControl(new Follower(leaderID, true));
        if (Robot.isReal()) {
            m_sensor = new TimeOfFlight(sensorID);

        } else {
            m_sensorSim = new DigitalInput(0);
            PhysicsSim.getInstance().addTalonFX(m_leaderMotor, 0.001);
            PhysicsSim.getInstance().addTalonFX(m_followerMotor, 0.001);
        }
        SmartDashboard.putNumber("IntakeSubsystem/SetPoint" ,0);
    }

    public void setIntakeSpeed(double speed) {
        SmartDashboard.putNumber("IntakeSubsystem/SetPoint" , speed);
        m_leaderMotor.setControl(m_request.withVelocity(speed).withSlot(0));
    }

    public void stop() {
        setIntakeSpeed(0);
    }

    public double getSpeed() {
        return m_leaderMotor.getVelocity().getValue();
    }

    public double getVoltage() {
        return m_leaderMotor.getMotorVoltage().getValue();
    }
    
    public boolean getSensor() {
        if (Robot.isReal()) {
            return m_sensor.getRange() < TRIGGER_DISTANT;
        } else {
            return m_sensorSim.get();
        }
    
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setIntakeSpeed(3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeSubsystem/Velocity" ,getSpeed());
        SmartDashboard.putNumber("IntakeSubsystem/Voltage" ,getVoltage());
    }
}

