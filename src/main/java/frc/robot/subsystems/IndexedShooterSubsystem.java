package frc.robot.subsystems;

 import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

public class IndexedShooterSubsystem extends SubsystemBase{

    private static final double TRIGGER_DISTANT = 200;
    private TalonFX m_leftMotor; 
    private TalonFX m_rightMotor; 
    private TalonFX m_indexMotor; 
    private TimeOfFlight m_sensor;
    private DigitalInput m_sensorSim;

    private MotionMagicVelocityVoltage m_leftRequest = new MotionMagicVelocityVoltage(0);
    private MotionMagicVelocityVoltage m_rightRequest = new MotionMagicVelocityVoltage(0);
    private MotionMagicVelocityVoltage m_indexerRequest = new MotionMagicVelocityVoltage(0);

    private final SlotConfigs FLYWHEEL_PIDFF_CONFIG = new SlotConfigs()
        .withKP(1)
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(1);
    private final MotionMagicConfigs FLYWHEEL_MM_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1)
        .withMotionMagicJerk(1);
    private final double FLYWHEEL_GEAR_RATIO = 1;

    private final SlotConfigs INDEXER_PIDFF_CONFIG = new SlotConfigs()
        //PID
        .withKP(1)
        .withKI(0)
        .withKD(0)
        //FeedForward
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(1);

    
    private final MotionMagicConfigs INDEXER_MM_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1) // Motor target acceleration
        .withMotionMagicJerk(1); // Motor max acceleration rate of change
    private final double INDEXER_GEAR_RATIO = 1;

    public IndexedShooterSubsystem(int shooterID, int shooterFollwerID, int indexID, int sensorID, String CANbus) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_leftMotor = new TalonFX(shooterID,CANbus);
        m_rightMotor = new TalonFX(shooterFollwerID,CANbus); 
        m_indexMotor = new TalonFX(indexID,CANbus);

        m_rightMotor.setInverted(true); //TODO find out which motor we need to invert

        TalonFXConfiguration flywheel_config = new TalonFXConfiguration();
        flywheel_config.withSlot0(Slot0Configs.from(FLYWHEEL_PIDFF_CONFIG));
        flywheel_config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        flywheel_config.withMotionMagic(FLYWHEEL_MM_CONFIGS); // The Motion Magic configs

        m_leftMotor.getConfigurator().apply(flywheel_config);
        m_leftMotor.setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.getConfigurator().apply(flywheel_config);
        m_rightMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration indexer_config = new TalonFXConfiguration();
        indexer_config.withSlot0(Slot0Configs.from(INDEXER_PIDFF_CONFIG));
        indexer_config.Feedback.SensorToMechanismRatio = INDEXER_GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        indexer_config.withMotionMagic(INDEXER_MM_CONFIGS); // The Motion Magic configs

        m_indexMotor.getConfigurator().apply(indexer_config);
        m_indexMotor.setNeutralMode(NeutralModeValue.Brake);

        
        if (Robot.isReal()) {
            m_sensor = new TimeOfFlight(sensorID);
        } else {
            m_sensorSim = new DigitalInput(0);
            PhysicsSim.getInstance().addTalonFX(m_indexMotor, 0.001);
            PhysicsSim.getInstance().addTalonFX(m_leftMotor, 0.001);
            PhysicsSim.getInstance().addTalonFX(m_rightMotor, 0.001);
        }

        SmartDashboard.putNumber("IndexedShooter/LeftWheel/SetPoint" ,0);
        SmartDashboard.putNumber("IndexedShooter/RightWheel/SetPoint" ,0);
        SmartDashboard.putNumber("IndexedShooter/Indexer/SetPoint" ,0);
    }

    public void setMotorSpeed(double leftSpeed, double rightSpeed) {
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/SetPoint" ,leftSpeed);
        SmartDashboard.putNumber("IndexedShooter/RightWheel/SetPoint" ,rightSpeed);
        m_leftMotor.setControl(m_leftRequest.withVelocity(leftSpeed).withSlot(0));
        m_rightMotor.setControl(m_rightRequest.withVelocity(rightSpeed).withSlot(0));
    }

    public double getLeftMotorSpeed() {
        return m_leftMotor.getVelocity().getValue();
    }

    public double getRightMotorSpeed() {
        return m_rightMotor.getVelocity().getValue();
    }

    public double getLeftVoltage() {
        return m_leftMotor.getMotorVoltage().getValue();
    }

    public double getRightVoltage() {
        return m_rightMotor.getMotorVoltage().getValue();
    }

    public void stop() {
        setMotorSpeed(0, 0);
    }

    public void stopIndexer() {
        setIndexSpeed(0);
    }

    public void setIndexSpeed(double speed) {
        SmartDashboard.putNumber("IndexedShooter/Indexer/SetPoint" ,speed);
        m_indexMotor.setControl(m_indexerRequest.withVelocity(speed).withSlot(0)); //TODO: Figure out how to do percent out
    }

    public double getIndexVelocity() {
       return  m_indexMotor.getVelocity().getValueAsDouble();
    }

    public double getIndexVoltage() {
        return m_indexMotor.getMotorVoltage().getValueAsDouble();
    }

    public boolean getSensor() {
        if (Robot.isReal()) {
            return m_sensor.getRange() < TRIGGER_DISTANT;
        } else {
            return m_sensorSim.get();
        }
    }

    public StartEndCommand getShootTest() {
        return new StartEndCommand(() -> {setMotorSpeed(3,4);}, () -> {stop();}, this);
    }

    public StartEndCommand getIndexTest() {
        return new StartEndCommand(()-> {setIndexSpeed(0.5);}, () -> {stopIndexer();}, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/Velocity" ,getLeftMotorSpeed());
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/Voltage" ,getLeftVoltage());

        SmartDashboard.putNumber("IndexedShooter/RightWheel/Velocity" ,getRightMotorSpeed());
        SmartDashboard.putNumber("IndexedShooter/RightWheel/Voltage" ,getRightVoltage());

        SmartDashboard.putNumber("IndexedShooter/Indexer/Velocity" ,getIndexVelocity());
        SmartDashboard.putNumber("IndexedShooter/Indexer/Voltage" ,getIndexVoltage());
    }
}

