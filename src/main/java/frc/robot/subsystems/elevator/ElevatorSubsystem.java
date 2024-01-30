package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.PosSubsystemIO;

public class ElevatorSubsystem extends SubsystemBase {
    private PosSubsystemIO IO;
    private String elevatorName;

    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(
        int motor1ID, 
        int motor2ID, 
        String CANbus, 
        Slot0Configs slot0Configs,
        MotionMagicConfigs mmConfigs,
        ElevatorType elevator
        ) {
        this.IO = Robot.isReal() 
        ? new ElevatorIORobot(new TalonFX(motor1ID,CANbus), new TalonFX(motor2ID,CANbus),slot0Configs,mmConfigs,elevator)
        : new ElevatorIOSim(new TalonFX(motor1ID,CANbus), new TalonFX(motor2ID,CANbus),slot0Configs,mmConfigs,elevator);
        elevatorName = "" + this.hashCode();
    } 

    /**
     * <h3>setTargetHeight</h3>
     * Sets the target height of the elevator
     * @param targetHeight The height the robot will try to move to
     */
    public void setTargetHeight(double targetHeight) {
        IO.setTarget(targetHeight);
    }

    /**
     * <h3>getHeight</h3>
     * Gets the current height of the elevator
     * @return The height of the elevator
     */
    public double getHeight() {
        return IO.getPos();
    }

    /**
     * <h3>getTargetHeight</h3>
     * Gets the height the elevator is trying to move to.
     * @return The target height
     */
    public double getTargetHeight() {
        return IO.getTarget();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the current velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getVelocity() {
        return IO.getVelocity();
    }

    @Override
    public void periodic() {
        IO.runSim();
        SmartDashboard.putNumber("Elevator-" + elevatorName + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Elevator-" + elevatorName + "/Height", getHeight());
        SmartDashboard.putNumber("Elevator-" + elevatorName + "/SetPoint", IO.getTarget());
        SmartDashboard.putNumber("Elevator-" + elevatorName + "/Voltage", IO.getVoltage());
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setTargetHeight(5);System.out.println("Elevator Test Start");},()->{setTargetHeight(0);},this);
    }
}
