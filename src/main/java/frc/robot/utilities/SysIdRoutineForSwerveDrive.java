package frc.robot.utilities;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SwerveDrivetrainSubsystem;

public class SysIdRoutineForSwerveDrive extends SwerveDrivetrainSubsystem {

    public enum SysIdTypeOfTest {Translation, Steer, Rotation};

    public SysIdRoutineForSwerveDrive(SysIdTypeOfTest type, SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configureSysIdBindings(new CommandXboxController(1),type);
    }
    
    public SysIdRoutineForSwerveDrive(SysIdTypeOfTest type, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(type, driveTrainConstants, 0, modules);
    }

    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine m_SysIdRoutineTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state)->SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts)->setControl(translationCharacterization.withVolts(volts)),
                null,
                this));

    private SysIdRoutine m_SysIdRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state)->SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts)->setControl(rotationCharacterization.withVolts(volts)),
                null,
                this));
    private SysIdRoutine m_SysIdRoutineSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state)->SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts)->setControl(steerCharacterization.withVolts(volts)),
                null,
                this));

    /* Both the sysid commands are specific to one particular sysid routine, change which one you're trying to characterize */
    public Command sysIdQuasistatic(SysIdTypeOfTest type, SysIdRoutine.Direction direction) {
        switch (type) {
            case Translation:
                return m_SysIdRoutineTranslation.quasistatic(direction);        
            case Steer:
                return m_SysIdRoutineSteer.quasistatic(direction);
            case Rotation:
                return m_SysIdRoutineRotation.quasistatic(direction);
            default:
                return m_SysIdRoutineTranslation.quasistatic(direction);        
        }
    }
    public Command sysIdDynamic(SysIdTypeOfTest type, SysIdRoutine.Direction direction) {
        switch (type) {
            case Translation:
                return m_SysIdRoutineTranslation.dynamic(direction);        
            case Steer:
                return m_SysIdRoutineSteer.dynamic(direction);
            case Rotation:
                return m_SysIdRoutineRotation.dynamic(direction);
            default:
                return m_SysIdRoutineTranslation.dynamic(direction);        
        }
    }

    public void configureSysIdBindings(CommandXboxController controller, SysIdTypeOfTest type) {
        /* Bindings for drivetrain characterization */
        /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
        controller.back().and(
        controller.y()
        )
        .whileTrue(sysIdDynamic(type, Direction.kForward));
        controller.back().and(
        controller.x()
        )
        .whileTrue(sysIdDynamic(type, Direction.kReverse));
        controller.start().and(
        controller.y()
        )
        .whileTrue(sysIdQuasistatic(type, Direction.kForward));
        controller.start().and(
        controller.x()
        )
        .whileTrue(sysIdQuasistatic(type, Direction.kReverse));


        /* Manually stop logging with left bumper after we're done with the tests */
        /* This isn't necessary, but is convenient to reduce the size of the hoot file */
        controller.leftBumper().onTrue(new RunCommand(SignalLogger::start));
        controller.rightBumper().onTrue(new RunCommand(SignalLogger::stop));
    
        // CREATE hoot file (signalLogger)
        // download and convert (using Tuner X Log Extractor) to wpilog file/sysid 2024 load file/ select columns
        // update PID/FF values switch to closed loop in teleop   .withDriveRequestType(DriveRequestType.Voltage);
    }

}
