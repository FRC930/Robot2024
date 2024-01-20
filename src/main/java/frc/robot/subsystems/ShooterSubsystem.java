package frc.robot.subsystems;

 import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem {

    private TalonFX m_leftMotor; 
    private TalonFX m_rightMotor; 

    public ShooterSubsystem(int shooterID, int shooterFollwerID) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_leftMotor = new TalonFX(shooterID);
        m_rightMotor = new TalonFX(shooterFollwerID); 

        m_rightMotor.setInverted(true); //TODO find out which motor we need to invert
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
