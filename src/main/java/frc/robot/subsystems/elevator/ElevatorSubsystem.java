package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.TalonWrapper;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO IO;

    private PIDController pidController;
    private ElevatorFeedforward ffController;

    private double targetHeight;


    /**
     * <h3>ElevatorSubsystem</h3>
     * Creates a subsystem representing the elevator on the robot.
     * @return
     */
    public ElevatorSubsystem(int motor1ID, int motor2ID, double maxHeight, String elevatorName) {
        this.IO = new ElevatorIORobot(new TalonWrapper(motor1ID), new TalonWrapper(motor2ID), 1, 1); //TODO: Fix values
        this.targetHeight = 0;

        this.pidController = new PIDController(30, 0, 0.3);
        this.ffController = new ElevatorFeedforward(0.0, 0.35, 0.0, 0.0); //TODO: Does not work for multi-elevator but we're revamping code on saturday so that's a problem for later us
    
        this.pidController.setTolerance(0.5, 0.5);
        this.pidController.calculate(0,targetHeight);
    }

    /**
     * <h3>setTargetHeight</h3>
     * Sets the target height of the elevator
     * @param targetHeight The height the robot will try to move to
     */
    public void setTargetHeight(double targetHeight) {
        this.targetHeight = targetHeight;
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
        return targetHeight;
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
        // Always runs if the robot is in sim, only runs IRL if robot is enabled.
        if(DriverStation.isEnabled() || !Robot.isReal()) {
            double effort = pidController.calculate(getHeight(), targetHeight);
            effort += ffController.calculate(IO.getCurrentVelocity());

            IO.setVoltage(effort);
        }
    }

    public StartEndCommand getTestCommand() {
        return new StartEndCommand(()->{setTargetHeight(5);},()->{setTargetHeight(0);},this);
    }

}
