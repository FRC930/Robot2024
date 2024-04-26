package frc.robot.utilities;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShotLoggingUtil {
    // PRESETS - - Preset logging vals so we don't have to do juggling. Keeps two static instances of the class to keep things global.
    private static final String[] logPathsTurret = {
        "/AdvantageKit/RealOutputs/SpeakerScoreUtility/distance",
        "/AdvantageKit/RealOutputs/AutoAim/HeadingOffset"
    };
    private static final String[] logPathsPivot = {
        "/AdvantageKit/RealOutputs/SpeakerScoreUtility/distance",
        "/AdvantageKit/RealOutputs/PivotSubsystem/Angle",
        "/AdvantageKit/RealOutputs/PivotSubsystem/SetPoint"
    };
    private static String lastShotResult = "Unknown";
    private static final String outputTurret = "logging/shotLogs/turret";
    private static final String outputPivot = "logging/shotLogs/pivot";
    private static ShotLoggingUtil pivotInstance;
    private static ShotLoggingUtil turretInstance;

    //FIELDS
    private static final NetworkTableInstance table = NetworkTableInstance.getDefault();
    private final String[] logPaths;
    private final String output;

    
    private ShotLoggingUtil(String[] logPaths, String output) {
        this.logPaths = logPaths;
        this.output = output;
    }

    //Get the value of a certain path on the networktable
    private static Object getTableVal(String path) {
        return table.getEntry(path).getValue().getValue();
    }

    //Gets whether or not a certain path exists
    private static boolean getTableValExists(String path) {
        return table.getEntry(path).exists();
    }

    //Gets the shot values that this util has been created with
    public static String[] getShotVals(String[] logPaths) {
        String[] logVals = new String[logPaths.length];

        for(int i = 0; i< logPaths.length; i++){
            String key = logPaths[i];
            if(getTableValExists(key)) {
                logVals[i] = getTableVal(key).toString();
            } else {
                logVals[i] = "INVALID";
            }
        }

        return logVals;
    }

    /**
     * Logs all the current specified shot values at the current time into the output
     * Also adds a comment onto the end.
     */
    public void logShot(String comment) {
        String[] pastOutput = (String[])Optional.ofNullable(getTableVal("/AdvantageKit/RealOutputs/" + this.output)).orElse(new String[0]);
        String[] newOutput = new String[pastOutput.length + 1];

        for(int i = 0; i < pastOutput.length; i++) {
            newOutput[i] = pastOutput[i];
        }
        
        if(comment == null) {
            comment = "none";
        }

        newOutput[newOutput.length - 1] = Logger.getTimestamp() + ", " + compileArrayToCSV(getShotVals(logPaths)) + ", " + comment
            .replace(' ','_')
            .replace(',','|');

        Logger.recordOutput(this.output, newOutput);
    }

    /**
     * Compiles an array to CSV format<p>
     * "arr[0], arr[1], arr[2]"
     * @param array
     * @return
     */
    private static String compileArrayToCSV(String[] array) {
        String out = "";
        for(int i = 0; i < array.length; i++) {
            if(i < array.length - 1) {
                out += array[i] + ", ";
            } else {
                out += array[i];
            }
        }
        return out;
    }

    //Compiles 
    public static String compileCSVFormattedStrings(String[] strings) {
        String out = "";
        for(int i = 0; i < strings.length; i++) {
            if(i < strings.length - 1) {
                out += strings[i] + ";";
            } else {
                out += strings[i] + "";
            }
        }
        return out;
    }

    private static void addShotResult(String result) {
        String[] pastOutput = (String[])Optional.ofNullable(getTableVal("/AdvantageKit/RealOutputs/" + "logging/shotsHit")).orElse(new String[0]);
        String[] newOutput = Arrays.copyOf(pastOutput, pastOutput.length + 1);
        newOutput[newOutput.length - 1] = lastShotResult;
        Logger.recordOutput("logging/shotsHit",newOutput);
    }

    public static void resolveLastShot(boolean didShotHit) {
        lastShotResult = didShotHit ? "Hit" : "Missed";
        addShotResult(lastShotResult);
    }

    public static void deleteLastShotLog() {
        String[] pastOutput = (String[])Optional.ofNullable(getTableVal("/AdvantageKit/RealOutputs/" + "logging/shotsHit")).orElse(new String[0]);
        String[] newOutput = Arrays.copyOf(pastOutput, pastOutput.length - 1);
        Logger.recordOutput("logging/shotsHit",newOutput);
    }

    public static void advanceToNextShot() {
        if(lastShotResult.equals("Unknown")) {
            addShotResult("Unknown");
        }
        lastShotResult = "Unknown";
    }

    public static ShotLoggingUtil getPivotInstance() {
        if(pivotInstance == null) {
            pivotInstance = new ShotLoggingUtil(logPathsPivot,outputPivot);
        }
        return pivotInstance;
    }

    public static ShotLoggingUtil getTurretInstance() {
        if(turretInstance == null) {
            turretInstance = new ShotLoggingUtil(logPathsTurret,outputTurret);
        }
        return turretInstance;
    }

    public static Command getLogWithCommand(ShotLoggingUtil util,String comment) {
        return new InstantCommand(() -> {
            util.logShot(comment);
        });
    }

    public Command getDoLogCommand(String comment) {
        return new InstantCommand(() -> {
            this.logShot(comment);
        });
    }

    public Command getDumpOutputCommand(String path) {
        return new InstantCommand(()->{
            Logger.recordOutput(path, compileCSVFormattedStrings((String[])Optional.ofNullable(getTableVal("/AdvantageKit/RealOutputs/" + this.output)).orElse(new String[0])));
        });
    }

    public static Command getAdvanceShotCommand() {
        return new InstantCommand(()->{
            advanceToNextShot();
        });
    }

    public static Command getAddShotCommand(boolean hit) {
        return new InstantCommand(()->{
            resolveLastShot(hit);
        });
    }

    public static Command getRemoveShotCommand() {
        return new InstantCommand(()->{
            deleteLastShotLog();
        });
    }
}