package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

import com.ctre.phoenix6.signals.NeutralModeValue;


/**
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

