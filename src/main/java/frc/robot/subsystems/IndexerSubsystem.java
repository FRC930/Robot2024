package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IOs.TalonRollerIO;
import frc.robot.IOs.TimeOfFlightIO;

/**
 * <h3>IndexerSubsystem</h3>
 * This subsystem controls the indexer
 */
public class IndexerSubsystem extends SubsystemBase {
    private TalonRollerIO roller;
    private TimeOfFlightIO tof;

    /**
     * <h3>IndexerSubsystem</h3>
     * Contains the indexer motor and time of flight through IOs, 
     * allowing it to take a physical motor or sim representation
     * @param motor RollerMotorIORobot (Physical) or RollerMotorIOSim (Simulation) for indexer motor
     * @param ToF TimeOfFlightIORobot (Physical) or TimeOfFlightIOSim (Simulation) for indexer sensor
     */
    public IndexerSubsystem(TalonRollerIO motor, TimeOfFlightIO ToF) {
        roller = motor;
        tof = ToF;

        motor.getTalon().setNeutralMode(NeutralModeValue.Brake); // Applies brake mode to belt
        
    }

    /**
     * <h3>setSpeed</h3>
     * @param speed PercentOutput speed to apply
     */
    public void setSpeed(double speed) {
        roller.setSpeed(speed);
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
        return roller.getSpeed();
    }

    /**
     * <h3>getVoltage</h3>
     * @return current applied voltage to Talon
     */
    public double getVoltage() {
        return roller.getVoltage();
    }

    /**
     * <h3>getSensor</h3>
     * @return value of indexer sensor
     */
    public boolean getSensor() {
        return tof.get();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/Velocity" ,getSpeed());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Voltage" ,getVoltage());
        Logger.recordOutput(this.getClass().getSimpleName() + "/Sensor", getSensor());
    }


}
