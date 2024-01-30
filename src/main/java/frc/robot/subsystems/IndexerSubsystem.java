package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerMotorIO;
import frc.robot.subsystems.timeofflight.TimeOfFlightIO;

public class IndexerSubsystem extends SubsystemBase {
    private RollerMotorIO roller;
    private TimeOfFlightIO tof;

    // private MotionMagicExpoVoltage m_request;

    public IndexerSubsystem(RollerMotorIO motor, TimeOfFlightIO ToF) {
        roller = motor;
        tof = ToF;

        // m_request = new MotionMagicExpoVoltage(0).withEnableFOC(true);

        // TalonFXConfiguration cfg = new TalonFXConfiguration();
        // cfg.withSlot0(Slot0Configs.from(slotConfigs));
        // cfg.withMotionMagic(mmConfigs);
        // cfg.Feedback.RotorToSensorRatio = gearRatio;
        
        // motor.getTalon().getConfigurator().apply(cfg);
        motor.getTalon().setNeutralMode(NeutralModeValue.Brake);
        
    }

    public void setSpeed(double speed) {
        roller.setSpeed(speed);
    }
    public void stop() {
        setSpeed(0);
    }

    public double getSpeed() {
        return roller.getSpeed();
    }

    public double getVoltage() {
        return roller.getVoltage();
    }

    public boolean getSensor() {
        return tof.get();
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(() -> {setSpeed(0.3);}, () -> {stop();}, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeSubsystem/Velocity" ,getSpeed());
        SmartDashboard.putNumber("IntakeSubsystem/Voltage" ,getVoltage());
        SmartDashboard.putBoolean("IntakeSubsystem/IntookenYet", getSensor());
    }


}
