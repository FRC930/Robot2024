package frc.robot.commands.Orchestra;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public interface Orchestratable {
    public TalonFX[] getInstruments();
    public default Subsystem[] getSubsystems() {
        return new Subsystem[0];
    };
    public default Supplier<Boolean> getShouldOrchestrate() {
        return () -> true;
    }
}
