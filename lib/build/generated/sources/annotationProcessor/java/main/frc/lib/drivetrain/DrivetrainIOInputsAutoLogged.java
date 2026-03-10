package frc.lib.drivetrain;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DrivetrainIOInputsAutoLogged extends DrivetrainIO.DrivetrainIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GyroYawDeg", gyroYawDeg);
    table.put("GyroRateDegPerSec", gyroRateDegPerSec);
    table.put("OdometryPeriodSec", odometryPeriodSec);
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DriveVelocityMps", driveVelocityMps);
    table.put("SteerAngleDeg", steerAngleDeg);
    table.put("DriveTempC", driveTempC);
    table.put("SteerTempC", steerTempC);
    table.put("DriveCurrentA", driveCurrentA);
    table.put("SteerCurrentA", steerCurrentA);
    table.put("BatteryVoltage", batteryVoltage);
    table.put("VisionConnected", visionConnected);
    table.put("VisionHasEstimate", visionHasEstimate);
    table.put("VisionPoseX", visionPoseX);
    table.put("VisionPoseY", visionPoseY);
    table.put("VisionPoseRotDeg", visionPoseRotDeg);
    table.put("VisionTimestampSec", visionTimestampSec);
    table.put("VisionTagCount", visionTagCount);
    table.put("VisionAmbiguity", visionAmbiguity);
    table.put("VisionAvgTagDistM", visionAvgTagDistM);
    table.put("VisionPoseZ", visionPoseZ);
  }

  @Override
  public void fromLog(LogTable table) {
    gyroYawDeg = table.get("GyroYawDeg", gyroYawDeg);
    gyroRateDegPerSec = table.get("GyroRateDegPerSec", gyroRateDegPerSec);
    odometryPeriodSec = table.get("OdometryPeriodSec", odometryPeriodSec);
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    driveVelocityMps = table.get("DriveVelocityMps", driveVelocityMps);
    steerAngleDeg = table.get("SteerAngleDeg", steerAngleDeg);
    driveTempC = table.get("DriveTempC", driveTempC);
    steerTempC = table.get("SteerTempC", steerTempC);
    driveCurrentA = table.get("DriveCurrentA", driveCurrentA);
    steerCurrentA = table.get("SteerCurrentA", steerCurrentA);
    batteryVoltage = table.get("BatteryVoltage", batteryVoltage);
    visionConnected = table.get("VisionConnected", visionConnected);
    visionHasEstimate = table.get("VisionHasEstimate", visionHasEstimate);
    visionPoseX = table.get("VisionPoseX", visionPoseX);
    visionPoseY = table.get("VisionPoseY", visionPoseY);
    visionPoseRotDeg = table.get("VisionPoseRotDeg", visionPoseRotDeg);
    visionTimestampSec = table.get("VisionTimestampSec", visionTimestampSec);
    visionTagCount = table.get("VisionTagCount", visionTagCount);
    visionAmbiguity = table.get("VisionAmbiguity", visionAmbiguity);
    visionAvgTagDistM = table.get("VisionAvgTagDistM", visionAvgTagDistM);
    visionPoseZ = table.get("VisionPoseZ", visionPoseZ);
  }

  public DrivetrainIOInputsAutoLogged clone() {
    DrivetrainIOInputsAutoLogged copy = new DrivetrainIOInputsAutoLogged();
    copy.gyroYawDeg = this.gyroYawDeg;
    copy.gyroRateDegPerSec = this.gyroRateDegPerSec;
    copy.odometryPeriodSec = this.odometryPeriodSec;
    copy.drivePositionRad = this.drivePositionRad.clone();
    copy.driveVelocityMps = this.driveVelocityMps.clone();
    copy.steerAngleDeg = this.steerAngleDeg.clone();
    copy.driveTempC = this.driveTempC.clone();
    copy.steerTempC = this.steerTempC.clone();
    copy.driveCurrentA = this.driveCurrentA.clone();
    copy.steerCurrentA = this.steerCurrentA.clone();
    copy.batteryVoltage = this.batteryVoltage;
    copy.visionConnected = this.visionConnected.clone();
    copy.visionHasEstimate = this.visionHasEstimate.clone();
    copy.visionPoseX = this.visionPoseX.clone();
    copy.visionPoseY = this.visionPoseY.clone();
    copy.visionPoseRotDeg = this.visionPoseRotDeg.clone();
    copy.visionTimestampSec = this.visionTimestampSec.clone();
    copy.visionTagCount = this.visionTagCount.clone();
    copy.visionAmbiguity = this.visionAmbiguity.clone();
    copy.visionAvgTagDistM = this.visionAvgTagDistM.clone();
    copy.visionPoseZ = this.visionPoseZ.clone();
    return copy;
  }
}
