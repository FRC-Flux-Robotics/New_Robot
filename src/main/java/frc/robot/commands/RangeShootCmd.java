package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.robot.FieldPositions;
import frc.robot.MechanismTuning;
import frc.robot.RangeTable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Automatic range-based shooting. Continuously calculates distance to hub, sets shooter speed and
 * hood angle from range table, and feeds when shooter reaches target speed.
 */
public class RangeShootCmd extends Command {
  private final VelocityMechanism m_shooter;
  private final PositionMechanism m_hood;
  private final VelocityMechanism m_feeder;
  private final VelocityMechanism m_indexer;
  private final RangeTable m_rangeTable;
  private final Supplier<Pose2d> m_poseSupplier;
  private final double m_timeoutSeconds;
  private final Timer m_timer = new Timer();

  /**
   * @param shooter shooter mechanism (dual motor)
   * @param hood hood position mechanism
   * @param feeder feeder mechanism
   * @param indexer indexer mechanism
   * @param rangeTable lookup table for speed/elevation by distance
   * @param poseSupplier current robot pose supplier
   * @param timeoutSeconds auto-finish timeout (0 = no timeout)
   */
  public RangeShootCmd(
      VelocityMechanism shooter,
      PositionMechanism hood,
      VelocityMechanism feeder,
      VelocityMechanism indexer,
      RangeTable rangeTable,
      Supplier<Pose2d> poseSupplier,
      double timeoutSeconds) {
    m_shooter = shooter;
    m_hood = hood;
    m_feeder = feeder;
    m_indexer = indexer;
    m_rangeTable = rangeTable;
    m_poseSupplier = poseSupplier;
    m_timeoutSeconds = timeoutSeconds;
    addRequirements(shooter, hood, feeder, indexer);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    Pose2d pose = m_poseSupplier.get();
    Pose2d hubPose = FieldPositions.resolve("HUB");
    if (hubPose == null) return;

    Translation2d hubPos = hubPose.getTranslation();
    double distance = pose.getTranslation().getDistance(hubPos);

    RangeTable.Range range = m_rangeTable.getRange(distance);
    if (range == null) return;

    m_shooter.setVelocity(range.speed);
    m_hood.setPosition(range.elevation);

    Logger.recordOutput("RangeShoot/Distance", distance);
    Logger.recordOutput("RangeShoot/SpeedRPS", range.speed);
    Logger.recordOutput("RangeShoot/Hood", range.elevation);
    Logger.recordOutput("RangeShoot/AtTarget", m_shooter.atTarget());

    if (m_shooter.atTarget()) {
      m_feeder.setVelocity(MechanismTuning.feederSpeed());
      m_indexer.setVelocity(MechanismTuning.indexerSpeed());
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_feeder.stop();
    m_indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timeoutSeconds > 0 && m_timer.hasElapsed(m_timeoutSeconds);
  }
}
