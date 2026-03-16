package frc.lib.drivetrain;

public class ModuleConfig {
  public final int driveMotorId;
  public final int steerMotorId;
  public final int encoderId;
  public final double encoderOffset;
  public final double xPositionInches;
  public final double yPositionInches;
  public final boolean invertDrive;
  public final boolean invertSteer;

  public ModuleConfig(
      int driveMotorId,
      int steerMotorId,
      int encoderId,
      double encoderOffset,
      double xPositionInches,
      double yPositionInches,
      boolean invertDrive,
      boolean invertSteer) {
    this.driveMotorId = driveMotorId;
    this.steerMotorId = steerMotorId;
    this.encoderId = encoderId;
    this.encoderOffset = encoderOffset;
    this.xPositionInches = xPositionInches;
    this.yPositionInches = yPositionInches;
    this.invertDrive = invertDrive;
    this.invertSteer = invertSteer;
  }
}
