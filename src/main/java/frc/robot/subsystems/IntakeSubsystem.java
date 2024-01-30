package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerMotorIO;
import frc.robot.subsystems.timeofflight.TimeOfFlightIO;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


/**
 * This subsystem controls the intake
 */
public class IntakeSubsystem extends SubsystemBase {

    private RollerMotorIO m_leaderMotor;
    //private MotionMagicVelocityVoltage m_request;
    private RollerMotorIO m_followerMotor;
    private TimeOfFlightIO m_sensorL;
    private TimeOfFlightIO m_sensorR;
    /* 
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
    */


    /**
     * 
     * This subsystem controls the intake
     * 
     * @param leaderID The ID to the leader motor
     * @param followerID The ID to the follower motor
     * @param sensorID 
     */
    public IntakeSubsystem(RollerMotorIO leader, RollerMotorIO follower, TimeOfFlightIO leftSensor, TimeOfFlightIO rightSensor)  {
        m_leaderMotor = leader;
        m_followerMotor = follower;
        m_sensorL = leftSensor;
        m_sensorR = rightSensor;


        /*m_request = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.withSlot0(Slot0Configs.from(PIDFF_CONFIG));
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO; //The ratio between the motor turning and the elevator moving. We may have to invert this
        config.withMotionMagic(MM_CONFIGS); // The Motion Magic configs
        

        m_leaderMotor.getConfigurator().apply(config);
        m_followerMotor.getConfigurator().apply(config);
        */
        m_followerMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);
        m_leaderMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

        m_followerMotor.getTalon().setControl(new Follower(m_leaderMotor.getTalon().getDeviceID(), true));
        SmartDashboard.putNumber("IntakeSubsystem/SetPoint", 0);
    }

    public void setIntakeSpeed(double speed) {
        m_leaderMotor.setSpeed(speed);
    }

    public void stop() {
        setIntakeSpeed(0);
    }

    public double getSpeed() {
        return m_leaderMotor.getSpeed();
    }

    public double getVoltage() {
        return m_leaderMotor.getVoltage();
    }

    public boolean getSensor() {
        return m_sensorL.get() || m_sensorR.get();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setIntakeSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        m_leaderMotor.runSim();
        m_followerMotor.runSim();
        SmartDashboard.putNumber("IntakeSubsystem/Velocity" ,getSpeed());
        SmartDashboard.putNumber("IntakeSubsystem/Voltage" ,getVoltage());
        SmartDashboard.putBoolean("IntakeSubsystem/IntookenYet", getSensor());
    }
}

