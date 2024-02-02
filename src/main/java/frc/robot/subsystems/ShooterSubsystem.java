package frc.robot.subsystems;

 import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import org.littletonrobotics.junction.Logger;

/**
 * <h3>ShooterSubsystem</h3>
 * This subsystem controls the shooter
 */
public class ShooterSubsystem extends SubsystemBase{

    private TalonRollerIO m_leftMotor; 
    private TalonRollerIO m_rightMotor;

    public ShooterSubsystem(TalonRollerIO leftMotor, TalonRollerIO rightMotor) {
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;

        m_rightMotor.getTalon().setInverted(true);

        // Applies coast mode to Talons
        m_leftMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

    }
    
    /**
    * <h3>setMotorSpeec</h3>
    * @param leftSpeed the speed the left motor will be set to
    * @param rightSpeed the speed the right motor will be set to
    */
    public void setMotorSpeed(double leftSpeed, double rightSpeed) {
        m_leftMotor.setSpeed(leftSpeed);
        m_rightMotor.setSpeed(rightSpeed);
    }

    /**
    * <h3>getLeftMotorSpeed</h3>
    * @return The current motor speed of the left motor
    */
    public double getLeftMotorSpeed() {
        return m_leftMotor.getSpeed();
    }

    /**
    * <h3>getRightMotorSpeed</h3>
    * @return The current motor speed of the right motor
    */
    public double getRightMotorSpeed() {
        return m_rightMotor.getSpeed();
    }

    /**
    * <h3>getLeftVoltage</h3>
    * @return The current voltage of the left motor
    */
    public double getLeftVoltage() {
        return m_leftMotor.getVoltage();
    }

    /**
    * <h3>getRightVoltage</h3>
    * @return The current voltage of the right motor
    */
    public double getRightVoltage() {
        return m_rightMotor.getVoltage();
    }

    /**
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    public void stop() {
        setMotorSpeed(0, 0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Velocity" ,getLeftMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LeftWheel/Voltage" ,getLeftVoltage());

        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Velocity" ,getRightMotorSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/RightWheel/Voltage" ,getRightVoltage());
    }
}

