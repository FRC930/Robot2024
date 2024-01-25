package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO IO;
    private String elevatorName;


    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(int motor1ID, int motor2ID, double gearRatio, double maxHeight,Slot0Configs slot0Configs,MotionMagicConfigs mmConfigs) {
        this.IO = new ElevatorIORobot(new TalonFX(motor1ID), new TalonFX(motor2ID),gearRatio,maxHeight,slot0Configs,mmConfigs);
        elevatorName = "" + this.hashCode();
    } //TODO: Pass in CAN id

    /**
     * <h3>setTargetHeight</h3>
     * Sets the target height of the elevator
     * @param targetHeight The height the robot will try to move to
     */
    public void setTargetHeight(double targetHeight) {
        IO.setTargetHeight(targetHeight);
    }

    /**
     * <h3>getHeight</h3>
     * Gets the current height of the elevator
     * @return The height of the elevator
     */
    public double getHeight() {
        return IO.getCurrentHeight();
    }

    /**
     * <h3>getTargetHeight</h3>
     * Gets the height the elevator is trying to move to.
     * @return The target height
     */
    public double getTargetHeight() {
        return IO.getTargetHeight();
    }

    /**
     * <h3>getVelocity</h3>
     * Gets the current velocity of the elevator
     * @return The velocity of the elevator
     */
    public double getVelocity() {
        return IO.getCurrentVelocity();
    }

    @Override
    public void periodic() {
        IO.updateInputs();
        SmartDashboard.putNumber("ElevatorVelocity-" + elevatorName, getVelocity());
        SmartDashboard.putNumber("ElevatorPosition-" + elevatorName, getHeight());
        SmartDashboard.putNumber("ElevatorSetpoint-" + elevatorName, IO.getTargetHeight());
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setTargetHeight(5);System.out.println("Elevator Test Start");},()->{setTargetHeight(0);},this);
    }
}
