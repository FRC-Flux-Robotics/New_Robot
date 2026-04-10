package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.robot.Vision;

/**
 * Fully automated camera calibration. Runs CameraValidationCmd repeatedly, applying corrections
 * between passes until all cameras converge or max iterations reached.
 *
 * <p>Typical run: 1-3 passes (~45-135 seconds). Each pass: 3 stations x 4 rotations with sampling.
 * Between passes: corrections applied automatically, 1 second settle time for transforms to
 * propagate.
 */
public class AutoCalibrateCmd extends Command {
  private static final int MAX_PASSES = 3;
  private static final double SETTLE_SECONDS = 1.0;
  private static final String PREFIX = "AutoCal/";

  private final Vision m_vision;
  private final DriveInterface m_drive;
  private final CameraConfig[] m_cameras;

  private CameraValidationCmd m_currentPass;
  private int m_passNumber;
  private boolean m_done;
  private boolean m_settling; // waiting for transforms to propagate after apply
  private final Timer m_settleTimer = new Timer();

  public AutoCalibrateCmd(Vision vision, DriveInterface drive, CameraConfig[] cameras) {
    m_vision = vision;
    m_drive = drive;
    m_cameras = cameras;
    addRequirements(vision, drive);
  }

  @Override
  public void initialize() {
    m_passNumber = 0;
    m_done = false;
    m_settling = false;
    SmartDashboard.putString(PREFIX + "Status", "Starting...");
    SmartDashboard.putNumber(PREFIX + "Pass", 0);
    startNextPass();
  }

  private void startNextPass() {
    m_passNumber++;
    SmartDashboard.putNumber(PREFIX + "Pass", m_passNumber);
    SmartDashboard.putString(
        PREFIX + "Status", String.format("Pass %d/%d — calibrating...", m_passNumber, MAX_PASSES));

    m_currentPass = new CameraValidationCmd(m_vision, m_drive, m_cameras);
    m_currentPass.initialize();
  }

  @Override
  public void execute() {
    if (m_done) return;

    // Settling: wait for transforms to propagate after applying corrections
    if (m_settling) {
      m_drive.drive(0, 0, 0, false, 0.02);
      if (m_settleTimer.hasElapsed(SETTLE_SECONDS)) {
        m_settling = false;
        startNextPass();
      }
      return;
    }

    // Run the current validation pass
    m_currentPass.execute();

    if (m_currentPass.isFinished()) {
      m_currentPass.end(false);

      CameraValidationCmd.Result result = m_currentPass.getResult();

      if (result == CameraValidationCmd.Result.ALL_PASSED) {
        SmartDashboard.putString(
            PREFIX + "Status",
            String.format("DONE — all cameras passed on pass %d/%d!", m_passNumber, MAX_PASSES));
        m_done = true;
        return;
      }

      if (m_passNumber >= MAX_PASSES) {
        SmartDashboard.putString(
            PREFIX + "Status",
            String.format(
                "DONE — %d passes complete, still not converged. Check CamVal results.",
                MAX_PASSES));
        m_done = true;
        return;
      }

      // Apply corrections and start next pass
      SmartDashboard.putString(
          PREFIX + "Status",
          String.format(
              "Pass %d/%d — %s, applying corrections...", m_passNumber, MAX_PASSES, result.name()));
      CameraValidationCmd.checkAutoApply(m_vision.getCameraCount());

      // Trigger CamTune/Apply so Vision.periodic() pushes transforms to VisionIO
      SmartDashboard.putBoolean("CamTune/Apply", true);

      // Wait for transforms to take effect before next pass
      m_settling = true;
      m_settleTimer.restart();
    }
  }

  @Override
  public boolean isFinished() {
    return m_done;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);

    if (interrupted) {
      // Let the current pass clean up if it was running
      if (m_currentPass != null && !m_currentPass.isFinished()) {
        m_currentPass.end(true);
      }
      SmartDashboard.putString(PREFIX + "Status", "Interrupted on pass " + m_passNumber);
    } else if (m_currentPass != null
        && m_currentPass.getResult() == CameraValidationCmd.Result.ALL_PASSED) {
      // Auto-save on success
      SmartDashboard.putBoolean("CamTune/Save", true);
      SmartDashboard.putString(
          PREFIX + "Status",
          String.format("DONE — converged on pass %d, saved to Preferences!", m_passNumber));
    }
  }
}
