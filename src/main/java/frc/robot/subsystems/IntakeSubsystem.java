package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This subsystem controls the intake
 */
public class IntakeSubsystem {

    private static final double TRIGGER_DISTANT = 200;
    private TalonFX m_leaderMotor;
    private TalonFX m_followerMotor;
    private TimeOfFlight m_sensor;
    private DigitalInput m_sensorSim;

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
        m_followerMotor.setControl(new Follower(leaderID, true));
        if (Robot.isReal()) {
            m_sensor = new TimeOfFlight(sensorID);

        } else {
            m_sensorSim = new DigitalInput(0);
        }
    }

    public void setIntakeSpeed(double speed) {
        m_leaderMotor.set(speed);
    }

    public void stop() {
        m_leaderMotor.set(0.0);
    }
    
    public boolean getSensor() {
        if (Robot.isReal()) {
            return m_sensor.getRange() < TRIGGER_DISTANT;
        } else {
            return m_sensorSim.get();
        }
    
    }
}

