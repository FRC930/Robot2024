package frc.robot.subsystems;

 import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.SparkMaxWrapper;
@Deprecated
public class SparkMaxShooterSubsystem {

    private SparkMaxWrapper m_topMotor; 
    private SparkMaxWrapper m_bottomMotor; 
   
    public SparkMaxShooterSubsystem(int shooterID, int shooterFollwerID) { //use IDs 3 & 4
        m_topMotor = new SparkMaxWrapper(shooterID, MotorType.kBrushless);
        m_bottomMotor = new SparkMaxWrapper(shooterFollwerID, MotorType.kBrushless); 

        m_topMotor.restoreFactoryDefaults();
        m_bottomMotor.restoreFactoryDefaults();

        m_bottomMotor.setInverted(true);
        // m_bottomMotor.follow(m_topMotor, true);//Neither of these motors should be inverted.
        
    }

    public void setMotorSpeed(double topSpeed, double bottomSpeed) {
        m_topMotor.set(topSpeed);
        m_bottomMotor.set(bottomSpeed);
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
