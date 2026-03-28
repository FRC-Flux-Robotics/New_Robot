package frc.robot;

/** Reasons a vision measurement can be rejected from pose estimation. */
public enum VisionRejectReason {
  TOO_AMBIGUOUS,
  OUT_OF_FIELD,
  Z_ERROR,
  NO_TAGS,
  ANGULAR_VEL_TOO_HIGH,
  MOVING_TOO_FAST
}
