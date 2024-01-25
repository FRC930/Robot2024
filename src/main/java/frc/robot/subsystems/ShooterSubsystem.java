package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_leftMotor; 
    private TalonFX m_rightMotor; 

    /* 
    private MotionMagicVelocityVoltage m_request;
    
    private final SlotConfigs PIDFF_CONFIG = new SlotConfigs()
        //PID
        .withKP(1)
        .withKI(0)
        .withKD(0)
        //FeedForward
        .withKA(0)
        .withKG(0)
        .withKS(0)
        .withKV(0);

    private final MotionMagicConfigs MM_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1) // Motor target acceleration
        .withMotionMagicJerk(1); // Motor max acceleration rate of change
        
    private final double GEAR_RATIO = 1;
    */


    public ShooterSubsystem(int shooterID, int shooterFollwerID) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_leftMotor = new TalonFX(shooterID);
        m_rightMotor = new TalonFX(shooterFollwerID);
        
        m_rightMotor.setInverted(true); //TODO find out which motor we need to invert

        /* 
        m_request = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withSlot0(Slot0Configs.from(PIDFF_CONFIG));
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        config.withMotionMagic(MM_CONFIGS); // The Motion Magic configs
        
        m_leftMotor.getConfigurator().apply(config);
        m_leftMotor.setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.getConfigurator().apply(config);
        m_rightMotor.setNeutralMode(NeutralModeValue.Coast);
        */
    }

    public void setMotorSpeed(double leftSpeed, double rightSpeed) {
        m_leftMotor.set(leftSpeed);
        m_rightMotor.set(rightSpeed);
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
        m_leftMotor.set(0.0);
        m_rightMotor.set(0.0);
    }

}
