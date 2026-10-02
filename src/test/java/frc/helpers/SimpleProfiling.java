package frc.helpers;

import java.time.Duration;
import java.util.function.Consumer;

import static frc.helpers.Ansi.*;

public class SimpleProfiling {

    @FunctionalInterface
    public interface ThrowingSupplier<T> {
        T get() throws Throwable;
    }

    /**
     * Profiles a function and passes the elapsed time to a consumer,
     * returning the direct result of the function.
     */
    private static <T> T profileFunction(String message, ThrowingSupplier<T> supplier, Consumer<Duration> timeConsumer) {
        long start = System.nanoTime();
        try {
            T result = supplier.get();
            long timeElapsed = System.nanoTime() - start;

            var currentMethodName = StackWalker.getInstance()
                    .walk(frames -> frames.findFirst().map(StackWalker.StackFrame::getMethodName).orElse("Unknown"));
            var parentMethodName = StackWalker.getInstance()
                    .walk(frames -> frames.filter(f -> !f.getMethodName().equals(currentMethodName)).findFirst().map(StackWalker.StackFrame::getMethodName).orElse("Unknown"));

            if(timeConsumer == null) {
                System.out.println(GREEN + parentMethodName + ": " + message + " (Elapsed time " + timeElapsed + "ns)" + RESET);
            } else {
                timeConsumer.accept(Duration.ofNanos(timeElapsed));
            }
            return result;
        } catch (Throwable ex) {
            throwAsUncheckedException(ex);
            return null; // Unreachable
        }
    }

    public static <T> T profileFunction(ThrowingSupplier<T> supplier, Consumer<Duration> timeConsumer) {
        return profileFunction("", supplier, timeConsumer);
    }

    public static <T> T profileFunction(String message, ThrowingSupplier<T> supplier) {
        return profileFunction(message, supplier, null);
    }


    @SuppressWarnings("unchecked")
    private static <E extends Throwable> void throwAsUncheckedException(Throwable ex) throws E {
        throw (E) ex;
    }
}