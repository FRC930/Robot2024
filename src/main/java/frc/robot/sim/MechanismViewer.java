package frc.robot.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * <h3>MechanismViewer</h3>
 * Creates visual representation for various mechanisms based off of subsystem values
 */
public class MechanismViewer {
    private Mechanism2d m_mainAssembly;
    private Mechanism2d m_turretAssembly;

    private MechanismLigament2d m_shootingElevatorControl;
    private MechanismLigament2d m_climbingElevatorControl;
    private MechanismLigament2d m_pivotControl;
    private MechanismLigament2d m_turretControl;

    private MechanismRoot2d m_shootingElevatorRoot;
    private MechanismRoot2d m_climbingElevatorRoot;
    private MechanismRoot2d m_turretRoot;
    
    private PivotSubsystem m_pivot;
    private ElevatorSubsystem m_shootingElevator;
    private ElevatorSubsystem m_climbingElevator;
    private TurretSubsystem m_turret;

    /**
     * <h3>MechanismViewer</h3>
     * Creates visual representation for various mechanisms based off of subsystem values
     * 
     * @param pivot Pivot subsystem
     * @param shootingElevator Elevator subsystem used to elevate the shooter and pivot
     * @param climbingElevator Elevator subsystem used for endgame
     * @param turret Turret subsystem
     */
    public MechanismViewer(PivotSubsystem pivot,ElevatorSubsystem shootingElevator, ElevatorSubsystem climbingElevator, TurretSubsystem turret) {
        m_pivot = pivot;
        m_shootingElevator = shootingElevator;
        m_climbingElevator = climbingElevator;
        m_turret = turret;

        // Main assembly window
        m_mainAssembly = new Mechanism2d(Units.inchesToMeters(36), Units.inchesToMeters(24));//(Units.inchesToMeters(100), Units.inchesToMeters(100));

        // Start pos of shooting elevator
        m_shootingElevatorRoot = m_mainAssembly.getRoot("shooting_elevator", Units.inchesToMeters(7.35), Units.inchesToMeters(10));

        // Creates robot frame representation ligament
        m_mainAssembly.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 5, new Color8Bit(Color.kBlanchedAlmond))
        );

        // Start pos of climbing elevator
        m_climbingElevatorRoot = m_mainAssembly.getRoot("climbing_elevator",Units.inchesToMeters(26),Units.inchesToMeters(10));

        // Turret window
        m_turretAssembly = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(30));

        // Start pos of turret ligament
        m_turretRoot = m_turretAssembly.getRoot("turret", Units.inchesToMeters(15), Units.inchesToMeters(15));

        // Adds elevator to the robot in simulation at robot frame
        m_shootingElevatorControl =
            m_shootingElevatorRoot.append(
                new MechanismLigament2d(
                    "shootingElevator",
                    Units.inchesToMeters(10),
                    90,
                    15,
                    new Color8Bit(Color.kGray)
                )
            );
        
        // Applies pivot ligament to end of shooting elevator ligament
        m_pivotControl =
            m_shootingElevatorControl.append(
                new MechanismLigament2d(
                    "PivotAndShooter",
                    Units.inchesToMeters(10),
                    90,
                    10,
                    new Color8Bit(Color.kAzure)
                )
            );
        
        // Applies climbing elevator to robot base
        m_climbingElevatorControl = 
            m_climbingElevatorRoot.append(
                new MechanismLigament2d(
                    "climbingElevator",
                    Units.inchesToMeters(10),
                    90,
                    10,
                    new Color8Bit(Color.kGreen)
                )
            );

        // Applies turret ligament for turret window
        m_turretControl =
            m_turretRoot.append(
                new MechanismLigament2d(
                    "turret",
                    Units.inchesToMeters(10),
                    0,
                    10,
                    new Color8Bit(Color.kDarkOrange)
                )
            );
    }   

    public void periodic() {
        // Updates position of mechanisms
        m_climbingElevatorControl.setLength(m_climbingElevator.getHeight()+0.2);
        m_pivotControl.setAngle(-90+m_pivot.getPosition());
        m_shootingElevatorControl.setLength(Units.inchesToMeters(m_shootingElevator.getHeight())+0.2);
        m_turretControl.setAngle(m_turret.getPosition());

        // Logs to advantage kit
        Logger.recordOutput(this.getClass().getSimpleName()+"/mechanism2d", m_mainAssembly);
        Logger.recordOutput(this.getClass().getSimpleName()+"/turret2d", m_turretAssembly);
    }
}
