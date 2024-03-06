package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

/**
 * <h3>IndexerSubsystem</h3>
 * This subsystem controls the indexer
 */
public class IndexerSubsystem extends SubsystemBase {
    private TalonRollerIO m_rollerIO;
    private TimeOfFlightIO m_sensorIO;
    private boolean m_sensorStatus;

    /**
     * <h3>IndexerSubsystem</h3>
     * Contains the indexer motor and time of flight through IOs, 
     * allowing it to take a physical motor or sim representation
     * @param motor RollerMotorIORobot (Physical) or RollerMotorIOSim (Simulation) for indexer motor
     * @param ToF TimeOfFlightIORobot (Physical) or TimeOfFlightIOSim (Simulation) for indexer sensor
     */
    public IndexerSubsystem(TalonRollerIO motor, TimeOfFlightIO ToF) {
        m_rollerIO = motor;
        m_sensorIO = ToF;

        motor.getTalon().setNeutralMode(NeutralModeValue.Brake); // Applies brake mode to belt
        
    }

    /**
     * <h3>setSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setSpeed(double speed) {
        m_rollerIO.setSpeed(speed);
    }

    /**
     * <h3>stop</h3>
     * Sets speed of motor to 0
     */
    public void stop() {
        setSpeed(0);
    }

    /**
     * <h3>getSpeed</h3>
     * @return current velocity of indexer as reported by the Talon
     */
    public double getSpeed() {
        return m_rollerIO.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return m_rollerIO.getVoltage();
    }

    /**
     * <h3>getSensor</h3>
     * @return value of indexer sensor
     */
    public boolean getSensor() {
        return m_sensorStatus;
    }

    public double getSensorDistance() {
        return m_sensorIO.getRange();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        m_sensorStatus = m_sensorIO.get();
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Sensor", getSensor());
        Logger.recordOutput(this.getClass().getSimpleName() + "/LastSensorCheck", Timer.getFPGATimestamp());
        Logger.recordOutput(this.getClass().getSimpleName() + "/SensorRange", m_sensorIO.getRange());
    }

    public Command newSetSpeedCommand(double speed) {
        return new InstantCommand(() -> setSpeed(speed), this);
    }

    public Command newUntilNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.5);
        }
    }

    public Command newUntilNoNoteFoundCommand() {
        if(Robot.isReal()) {
            return new WaitUntilCommand(() -> !getSensor());  // DO not set subsystem since just getting sensor value
        } else {
            return new WaitCommand(.1);
        }
    }


}
