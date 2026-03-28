package frc.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class PhoenixUtilTest {

  @Test
  void succeedsOnFirstAttempt() {
    StatusCode result = PhoenixUtil.tryUntilOk(() -> StatusCode.OK, 5);
    assertTrue(result.isOK());
  }

  @Test
  void retriesUntilSuccess() {
    AtomicInteger count = new AtomicInteger(0);
    StatusCode result =
        PhoenixUtil.tryUntilOk(
            () -> {
              if (count.incrementAndGet() < 3) {
                return StatusCode.GeneralError;
              }
              return StatusCode.OK;
            },
            5);
    assertTrue(result.isOK());
    assertEquals(3, count.get());
  }

  @Test
  void returnsLastErrorAfterMaxAttempts() {
    AtomicInteger count = new AtomicInteger(0);
    StatusCode result =
        PhoenixUtil.tryUntilOk(
            () -> {
              count.incrementAndGet();
              return StatusCode.GeneralError;
            },
            5);
    assertFalse(result.isOK());
    assertEquals(5, count.get());
  }

  @Test
  void singleAttemptSucceeds() {
    StatusCode result = PhoenixUtil.tryUntilOk(() -> StatusCode.OK, 1);
    assertTrue(result.isOK());
  }

  @Test
  void singleAttemptFails() {
    StatusCode result = PhoenixUtil.tryUntilOk(() -> StatusCode.GeneralError, 1);
    assertFalse(result.isOK());
  }

  @Test
  void zeroAttemptsReturnsNotInitialized() {
    StatusCode result = PhoenixUtil.tryUntilOk(() -> StatusCode.OK, 0);
    assertFalse(result.isOK());
  }
}
