package frc.robot.subsystems;

 import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;

public class IndexedShooterSubsystem {

    private static final double TRIGGER_DISTANT = 200;
    private TalonFX m_leftMotor; 
    private TalonFX m_rightMotor; 
    private TalonFX m_indexMotor; 
    private TimeOfFlight m_sensor;
    private DigitalInput m_sensorSim;

    public IndexedShooterSubsystem(int shooterID, int shooterFollwerID, int indexID, int sensorID) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_leftMotor = new TalonFX(shooterID);
        m_rightMotor = new TalonFX(shooterFollwerID); 
        m_indexMotor = new TalonFX(indexID);

        m_rightMotor.setInverted(true); //TODO find out which motor we need to invert

        if (Robot.isReal()) {
            m_sensor = new TimeOfFlight(sensorID);

        } else {
            m_sensorSim = new DigitalInput(0);
        }
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

    public void setIndexSpeed(double speed) {
        m_indexMotor.set(speed);
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
}
