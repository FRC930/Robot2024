package frc.robot.sim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * <h3>MechanismViewer</h3>
 * Creates visual representation for various mechanisms based off of subsystem values
 */
public class MechanismViewer {
    private Mechanism2d m_mainAssembly;
    private Mechanism2d m_turretAssembly;

    private MechanismLigament2d m_pivotControl;
    private MechanismLigament2d m_turretControl;

    private MechanismLigament2d m_pivotControlTarget;
    private MechanismLigament2d m_turretControlTarget;

    private MechanismRoot2d m_pivotRoot;
    private MechanismRoot2d m_turretRoot;
    
    private PivotSubsystem m_pivot;
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
    public MechanismViewer(PivotSubsystem pivot, TurretSubsystem turret) {
        m_pivot = pivot;
        m_turret = turret;

        // Main assembly window
        m_mainAssembly = new Mechanism2d(Units.inchesToMeters(40), Units.inchesToMeters(40));//(Units.inchesToMeters(100), Units.inchesToMeters(100));

        // Start pos of shooting elevator
        m_pivotRoot = m_mainAssembly.getRoot("shooting_elevator", Units.inchesToMeters(7.35), Units.inchesToMeters(25));

        // Creates robot frame representation ligament
        m_mainAssembly.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 30, new Color8Bit(Color.kBlue))
        );

        // Turret window
        m_turretAssembly = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(30));

        // Start pos of turret ligament
        m_turretRoot = m_turretAssembly.getRoot("turret", Units.inchesToMeters(15), Units.inchesToMeters(15));

        // Applies pivot ligament to end of shooting elevator ligament
        m_pivotControl =
            m_pivotRoot.append(
                new MechanismLigament2d(
                    "PivotAndShooter",
                    Units.inchesToMeters(20),
                    180,
                    10,
                    new Color8Bit(Color.kAzure)
                )
            );

        // Applies pivot ligament target to end of shooting elevator ligament
        m_pivotControlTarget =
            m_pivotRoot.append(
                new MechanismLigament2d(
                    "PivotAndShooterTarget",
                    Units.inchesToMeters(20),
                    180,
                    2,
                    new Color8Bit(Color.kBlue)
                )
            );
        
        // Applies turret ligament for turret window
        m_turretControl =
            m_turretRoot.append(
                new MechanismLigament2d(
                    "turret",
                    Units.inchesToMeters(12),
                    0,
                    15,
                    new Color8Bit(Color.kDarkOrange)
                )
            );
        
        // Applies turret ligament for turret window
        m_turretControlTarget =
            m_turretRoot.append(
                new MechanismLigament2d(
                    "turretTarget",
                    Units.inchesToMeters(12),
                    0,
                    2,
                    new Color8Bit(Color.kBlue)
                )
            );
    }   

    public void periodic() {
        // Updates position of mechanisms
        // m_climbingElevatorControl.setLength(Units.inchesToMeters(m_climbingElevator.getPosition()+0.2));
        // m_climbingElevatorControlTarget.setLength(Units.inchesToMeters(m_climbingElevator.getTarget()+0.2));
        m_pivotControl.setAngle(m_pivot.getPosition());
        m_pivotControlTarget.setAngle(m_pivot.getTarget());
        // m_shootingElevatorControl.setLength(Units.inchesToMeters(m_shootingElevator.getPosition())+0.2);
        // m_shootingElevatorControlTarget.setLength(Units.inchesToMeters(m_shootingElevator.getTarget())+0.2);
        // Mech2d is counter-clockwise positive so negate values
        // https://github.com/wpilibsuite/frc-docs/blob/stable/source/docs/software/dashboards/glass/mech2d-widget.rst#id5
        m_turretControl.setAngle(-m_turret.getPosition());
        m_turretControlTarget.setAngle(-m_turret.getTarget());
       

        // Logs to advantage kit
        Logger.recordOutput(this.getClass().getSimpleName()+"/mechanism2d", m_mainAssembly);
        Logger.recordOutput(this.getClass().getSimpleName()+"/turret2d", m_turretAssembly);
    }
}
