package frc.robot.subsystems;

 import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;
public class SparkMaxShooterSubsystem {

    private CANSparkMax m_topMotor; 
    private CANSparkMax m_bottomMotor; 
    private final SparkMaxWrapper m_topMotorWrapper;
   
    public SparkMaxShooterSubsystem(int shooterID, int shooterFollwerID) { //use IDs 3 & 4 TODO check to make sure those IDs are free
        m_topMotor = new CANSparkMax(shooterID, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(shooterFollwerID, MotorType.kBrushless); 
        m_bottomMotor.follow(m_topMotor, false);//Neither of these motors should be inverted.
        m_topMotorWrapper = new SparkMaxWrapper(shooterID, MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed) {
        m_topMotor.set(speed);
    }

    public double gettopMotorSpeed() {
        return m_topMotor.getEncoder().getVelocity();
    }

    public double getbottomMotorSpeed() {
        return m_bottomMotor.getEncoder().getVelocity();
    }

    public double gettopVoltage() {
        return m_topMotor.getBusVoltage();
    }

    public double getbottomVoltage() {
        return m_bottomMotor.getBusVoltage();
    }

    public void stop() {
        m_topMotor.set(0.0);
        m_bottomMotor.set(0.0);
    }

}