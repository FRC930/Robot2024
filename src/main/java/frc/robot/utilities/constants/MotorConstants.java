package frc.robot.utilities.constants;


public class MotorConstants {
    private static MotorConstants instance;

    public final int KRAKEN_MAX_VOLTAGE = 24;

    private MotorConstants() {}

    public static MotorConstants getInstance() {
        if(instance == null) {
            instance = new MotorConstants();
        }
        return instance;
    }
}
