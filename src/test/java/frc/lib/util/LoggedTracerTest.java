package frc.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import org.junit.jupiter.api.Test;

/**
 * LoggedTracer is a thin wrapper over Timer.getFPGATimestamp() and Logger.recordOutput(), both of
 * which require HAL/AdvantageKit initialization. These tests verify the API contract and class
 * structure; runtime behavior is covered by integration tests.
 */
class LoggedTracerTest {

  @Test
  void classIsFinalAndNotInstantiable() throws Exception {
    assertTrue(Modifier.isFinal(LoggedTracer.class.getModifiers()));
    Constructor<?> ctor = LoggedTracer.class.getDeclaredConstructor();
    assertTrue(Modifier.isPrivate(ctor.getModifiers()));
  }

  @Test
  void resetMethodExists() throws Exception {
    Method reset = LoggedTracer.class.getMethod("reset");
    assertTrue(Modifier.isStatic(reset.getModifiers()));
    assertEquals(void.class, reset.getReturnType());
    assertEquals(0, reset.getParameterCount());
  }

  @Test
  void traceMethodExists() throws Exception {
    Method trace = LoggedTracer.class.getMethod("trace", String.class);
    assertTrue(Modifier.isStatic(trace.getModifiers()));
    assertEquals(void.class, trace.getReturnType());
    assertEquals(1, trace.getParameterCount());
  }

  @Test
  void onlyExposesResetAndTrace() {
    // Only public methods beyond Object should be reset() and trace(String)
    long publicMethodCount =
        java.util.Arrays.stream(LoggedTracer.class.getDeclaredMethods())
            .filter(m -> Modifier.isPublic(m.getModifiers()))
            .count();
    assertEquals(2, publicMethodCount);
  }
}
