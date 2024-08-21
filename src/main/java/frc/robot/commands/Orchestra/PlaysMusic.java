package frc.robot.commands.Orchestra;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PlaysMusic {
    public TalonFX[] getInstruments();
    public Subsystem[] getSubsystems();
}
