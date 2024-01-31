package frc.robot.subsystems;

 import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;

public class ShooterSubsystem extends SubsystemBase{

    private TalonRollerIO m_leftMotor; 
    private TalonRollerIO m_rightMotor;

    // private MotionMagicVelocityVoltage m_leftRequest = new MotionMagicVelocityVoltage(0);
    // private MotionMagicVelocityVoltage m_rightRequest = new MotionMagicVelocityVoltage(0);

    /* 
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
    */

    // private final SlotConfigs INDEXER_PIDFF_CONFIG = new SlotConfigs()
    //     //PID
    //     .withKP(1)
    //     .withKI(0)
    //     .withKD(0)
    //     //FeedForward
    //     .withKA(0)
    //     .withKG(0)
    //     .withKS(0)
    //     .withKV(1);

    
    // private final MotionMagicConfigs INDEXER_MM_CONFIGS = new MotionMagicConfigs()
    //     .withMotionMagicAcceleration(1) // Motor target acceleration
    //     .withMotionMagicJerk(1); // Motor max acceleration rate of change
    // private final double INDEXER_GEAR_RATIO = 1;

    public ShooterSubsystem(TalonRollerIO leftMotor, TalonRollerIO rightMotor) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;

        m_rightMotor.getTalon().setInverted(true);//TODO find out which motor we need to invert

        /* 
        TalonFXConfiguration flywheel_config = new TalonFXConfiguration();
        flywheel_config.withSlot0(Slot0Configs.from(FLYWHEEL_PIDFF_CONFIG));
        flywheel_config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        flywheel_config.withMotionMagic(FLYWHEEL_MM_CONFIGS); // The Motion Magic configs

        m_leftMotor.getConfigurator().apply(flywheel_config);
        m_rightMotor.getConfigurator().apply(flywheel_config);
        */
        m_leftMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

        /* 
        TalonFXConfiguration indexer_config = new TalonFXConfiguration();
        indexer_config.withSlot0(Slot0Configs.from(INDEXER_PIDFF_CONFIG));
        indexer_config.Feedback.SensorToMechanismRatio = INDEXER_GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        indexer_config.withMotionMagic(INDEXER_MM_CONFIGS); // The Motion Magic configs

        m_indexMotor.getConfigurator().apply(indexer_config);*/


        SmartDashboard.putNumber("IndexedShooter/LeftWheel/SetPoint" ,0);
        SmartDashboard.putNumber("IndexedShooter/RightWheel/SetPoint" ,0);
        SmartDashboard.putNumber("IndexedShooter/Indexer/SetPoint" ,0);
    }

    public void setMotorSpeed(double leftSpeed, double rightSpeed) {
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/SetPoint" ,leftSpeed);
        SmartDashboard.putNumber("IndexedShooter/RightWheel/SetPoint" ,rightSpeed);
        //m_leftMotor.setControl(m_leftRequest.withVelocity(leftSpeed).withSlot(0));
        //m_rightMotor.setControl(m_rightRequest.withVelocity(rightSpeed).withSlot(0));
        m_leftMotor.setSpeed(leftSpeed);
        m_rightMotor.setSpeed(rightSpeed);
    }

    public double getLeftMotorSpeed() {
        return m_leftMotor.getSpeed();
    }

    public double getRightMotorSpeed() {
        return m_rightMotor.getSpeed();
    }

    public double getLeftVoltage() {
        return m_leftMotor.getVoltage();
    }

    public double getRightVoltage() {
        return m_rightMotor.getVoltage();
    }

    public void stop() {
        setMotorSpeed(0, 0);
    }

    public StartEndCommand getShootTest() {
        return new StartEndCommand(() -> {setMotorSpeed(3,4);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/Velocity" ,getLeftMotorSpeed());
        SmartDashboard.putNumber("IndexedShooter/LeftWheel/Voltage" ,getLeftVoltage());

        SmartDashboard.putNumber("IndexedShooter/RightWheel/Velocity" ,getRightMotorSpeed());
        SmartDashboard.putNumber("IndexedShooter/RightWheel/Voltage" ,getRightVoltage());
    }
}

