package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Utility methods for CTRE Phoenix 6 hardware configuration. */
public final class PhoenixUtil {

  private PhoenixUtil() {}

  /**
   * Retries a CTRE configuration call until it succeeds or max attempts are exhausted.
   *
   * @param configCall supplier that performs the config and returns a StatusCode
   * @param maxAttempts maximum number of attempts (typically 5)
   * @return the last StatusCode (isOK() if any attempt succeeded)
   */
  public static StatusCode tryUntilOk(Supplier<StatusCode> configCall, int maxAttempts) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < maxAttempts; i++) {
      status = configCall.get();
      if (status.isOK()) {
        return status;
      }
    }
    return status;
  }
}
