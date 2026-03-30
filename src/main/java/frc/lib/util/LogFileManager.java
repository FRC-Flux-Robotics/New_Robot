package frc.lib.util;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

/** Manages AdvantageKit log file storage with automatic old-log cleanup. */
public class LogFileManager {
  private static final String LOG_PATH = "/home/lvuser/logs";
  private static final long MAX_STORAGE_BYTES = 500L * 1024 * 1024; // 500 MB

  private LogFileManager() {}

  /**
   * Returns the log directory path, creating it if needed and cleaning old logs if storage exceeds
   * the threshold.
   */
  public static String getLogPath() {
    return getLogPath(new File(LOG_PATH), MAX_STORAGE_BYTES);
  }

  /** Package-private overload for testing with custom directory and threshold. */
  static String getLogPath(File logDir, long maxBytes) {
    if (!logDir.exists()) {
      logDir.mkdirs();
    }
    cleanOldLogs(logDir, maxBytes);
    return logDir.getAbsolutePath();
  }

  /** Deletes oldest .wpilog files until total size is under the threshold. */
  static void cleanOldLogs(File logDir, long maxBytes) {
    File[] logFiles = logDir.listFiles((dir, name) -> name.endsWith(".wpilog"));
    if (logFiles == null || logFiles.length == 0) {
      return;
    }

    // Sort oldest first
    Arrays.sort(logFiles, Comparator.comparingLong(File::lastModified));

    long totalSize = 0;
    for (File f : logFiles) {
      totalSize += f.length();
    }

    int i = 0;
    while (totalSize > maxBytes && i < logFiles.length) {
      long fileSize = logFiles[i].length();
      if (logFiles[i].delete()) {
        System.out.println(
            "[LogFileManager] Deleted old log: "
                + logFiles[i].getName()
                + " ("
                + (fileSize / 1024 / 1024)
                + " MB)");
        totalSize -= fileSize;
      }
      i++;
    }
  }
}
