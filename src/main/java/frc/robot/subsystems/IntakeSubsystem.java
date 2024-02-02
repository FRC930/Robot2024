package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

import com.ctre.phoenix6.signals.NeutralModeValue;


/**
 * <h3>IntakeSubsystem</h3>
 * This subsystem controls the intake
 */
public class IntakeSubsystem extends SubsystemBase {

    private TalonRollerIO m_leaderMotor;
    private TalonRollerIO m_followerMotor;
    private TimeOfFlightIO m_sensorL;
    private TimeOfFlightIO m_sensorR;


    /**
     * 
     * This subsystem controls the intake
     * 
     * @param leaderID The ID to the leader motor
     * @param followerID The ID to the follower motor
     * @param sensorID 
     */
    public IntakeSubsystem(TalonRollerIO leader, TalonRollerIO follower, TimeOfFlightIO leftSensor, TimeOfFlightIO rightSensor)  {
        m_leaderMotor = leader;
        m_followerMotor = follower;
        m_sensorL = leftSensor;
        m_sensorR = rightSensor;

        m_followerMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);
        m_leaderMotor.getTalon().setNeutralMode(NeutralModeValue.Coast);

        m_followerMotor.getTalon().setControl(new Follower(m_leaderMotor.getTalon().getDeviceID(), true));
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", 0);
    }

    /**
    * <h3>setIntakeSpeed</h3>
    * @param speed the speed the motor will be set to
    */
    public void setIntakeSpeed(double speed) {
        m_leaderMotor.setSpeed(speed);
    }

    /**
    * <h3>stop</h3>
    * This sets the intake's speed to 0
    */
    public void stop() {
        setIntakeSpeed(0);
    }

    /**
    * <h3>getSpeed</h3>
    * @return the speed of the intake
    */
    public double getSpeed() {
        return m_leaderMotor.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return m_leaderMotor.getVoltage();
    }

    /**
     * <h3>getSensor</h3>
     * @return value of left sensor or right sensor (either one being true will return true)
     */
    public boolean getSensor() {
        return m_sensorL.get() || m_sensorR.get();
    }

    @Override
    public void periodic() {
        m_leaderMotor.runSim();
        m_followerMotor.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/IntookenYet", getSensor());
    }
}

