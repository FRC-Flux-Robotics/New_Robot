package frc.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LogFileManagerTest {

  @TempDir File tempDir;

  @Test
  void createsDirectoryIfMissing() {
    File subDir = new File(tempDir, "logs");
    assertFalse(subDir.exists());

    String result = LogFileManager.getLogPath(subDir, 500 * 1024 * 1024);

    assertTrue(subDir.exists());
    assertEquals(subDir.getAbsolutePath(), result);
  }

  @Test
  void deletesOldestLogsWhenOverThreshold() throws IOException {
    long maxBytes = 100; // 100 bytes threshold

    // Create 3 files: 50 bytes each = 150 total, over 100 threshold
    File oldest = createLogFile("old.wpilog", 50, 1000);
    File middle = createLogFile("mid.wpilog", 50, 2000);
    File newest = createLogFile("new.wpilog", 50, 3000);

    LogFileManager.cleanOldLogs(tempDir, maxBytes);

    // Oldest should be deleted to get under threshold (100 remaining = 100, at threshold)
    assertFalse(oldest.exists(), "oldest log should be deleted");
    assertTrue(middle.exists(), "middle log should be kept");
    assertTrue(newest.exists(), "newest log should be kept");
  }

  @Test
  void preservesNewestLogs() throws IOException {
    long maxBytes = 60;

    // 3 files of 50 bytes = 150 total, need to delete 2 to get under 60
    File oldest = createLogFile("a.wpilog", 50, 1000);
    File middle = createLogFile("b.wpilog", 50, 2000);
    File newest = createLogFile("c.wpilog", 50, 3000);

    LogFileManager.cleanOldLogs(tempDir, maxBytes);

    assertFalse(oldest.exists());
    assertFalse(middle.exists());
    assertTrue(newest.exists(), "newest log must be preserved");
  }

  @Test
  void leavesNonWpilogFilesUntouched() throws IOException {
    long maxBytes = 10; // Very low threshold

    File textFile = new File(tempDir, "notes.txt");
    Files.write(textFile.toPath(), new byte[100]);
    File logFile = createLogFile("test.wpilog", 50, 1000);

    LogFileManager.cleanOldLogs(tempDir, maxBytes);

    assertTrue(textFile.exists(), "non-wpilog files should not be deleted");
    assertFalse(logFile.exists(), "wpilog over threshold should be deleted");
  }

  @Test
  void handlesEmptyDirectory() {
    // Should not throw
    assertDoesNotThrow(() -> LogFileManager.cleanOldLogs(tempDir, 100));
  }

  private File createLogFile(String name, int sizeBytes, long lastModified) throws IOException {
    File file = new File(tempDir, name);
    Files.write(file.toPath(), new byte[sizeBytes]);
    file.setLastModified(lastModified);
    return file;
  }
}
