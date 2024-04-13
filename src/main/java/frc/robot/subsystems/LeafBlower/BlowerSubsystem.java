package frc.robot.subsystems.LeafBlower;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkMaxWrapper;

public class BlowerSubsystem extends SubsystemBase {

    private SparkMaxWrapper m_SparkMaxWrapper = new SparkMaxWrapper(19, MotorType.kBrushed);

    public BlowerSubsystem(){
        m_SparkMaxWrapper.resetToFactoryDefaults();
        m_SparkMaxWrapper.setSmartCurrentLimit(12);
        m_SparkMaxWrapper.setShouldBrake(false);
    }

    public void setSpeed(double speed){
        m_SparkMaxWrapper.set(speed);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getClass().getSimpleName() + "/speed", m_SparkMaxWrapper.get());
    }

    public Command getNewSetSpeedCommand(double speed) {
        return new InstantCommand(()->{
            setSpeed(speed);
        });
    }
}
