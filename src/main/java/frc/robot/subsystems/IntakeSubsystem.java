package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;
import frc.robot.utilities.Phoenix6Utility;

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

    private boolean justIntook = false;


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

        m_leaderMotor.getTalon().setInverted(true);

        Phoenix6Utility.applyConfigAndRetry(m_followerMotor.getTalon(), 
            () -> m_followerMotor.getTalon().setControl(new Follower(m_leaderMotor.getTalon().getDeviceID(), false)));
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", 0.0);
    }

    /**
    * <h3>setIntakeSpeed</h3>
    * @param speed the speed the motor will be set to
    */
    public void setSpeed(double speed) {
        m_leaderMotor.setSpeed(speed);
    }

    /**
    * <h3>stop</h3>
    * This sets the intake's speed to 0
    */
    public void stop() {
        setSpeed(0);
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

    public void setJustIntook(boolean set) {
        justIntook = set;
    }

    public boolean getJustIntook() {
        return justIntook;
    }

    public Command newSetJustIntookCommand(boolean set) {
        return new InstantCommand(() -> setJustIntook(set));
    }
 
    @Override
    public void periodic() {
        m_leaderMotor.runSim();
        m_followerMotor.runSim();
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LInputCurrent" ,m_leaderMotor.getInputCurrent());
        Logger.recordOutput(this.getClass().getSimpleName() + "/FInputCurrent" ,m_followerMotor.getInputCurrent());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LStatorCurrent" ,m_leaderMotor.getStatorCurrent());
        Logger.recordOutput(this.getClass().getSimpleName() + "/FStatorCurrent" ,m_followerMotor.getStatorCurrent());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/IntookenYet", getSensor());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SetPoint", m_leaderMotor.getTalon().get());
    }

    public InstantCommand newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }
}

