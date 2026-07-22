package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;

public class Common {
    private static final int MINIMAL_ITERATIONS = 5;
    private static BacktraceLevel backtraceLevel = null;

    public static <T> T todo() throws Error {
        Error e = new Error("not yet implemented");

        handler(e, System.err);

        System.exit(101);
        throw e;
    }

    public static <T> T todo(Telemetry telemetry) throws Error {
        telemetry.clearAll();
        telemetry.setItemSeparator("");
        telemetry.setCaptionValueSeparator("");

        Error e = new Error("not yet implemented");
        ByteArrayOutputStream out = new ByteArrayOutputStream();

        handler(e, new PrintStream(out, true));

        for (String line : out.toString(StandardCharsets.UTF_8).split("\n")) {
            telemetry.addData(line, "\n");
        }

        telemetry.update();

        System.exit(101);
        throw e;
    }

    public static <T> T todo(TelemetryManager telemetry) throws Error {
        return todo(telemetry.getWrapper());
    }

    private static void handler(Error e, PrintStream out) {
        if (backtraceLevel == null) {
            try {
                switch (System.getenv("JAVA_BACKTRACE")) {
                    case "0" -> backtraceLevel = BacktraceLevel.None;
                    case "full" -> backtraceLevel = BacktraceLevel.Full;
                    case null -> backtraceLevel = BacktraceLevel.None;
                    default -> backtraceLevel = BacktraceLevel.Minimal;
                }

            } catch (Exception ignored) {
                backtraceLevel = BacktraceLevel.None;
            }
        }

        StackTraceElement[] stack = e.getStackTrace();
        StackTraceElement stackTop = stack[0];

        out.printf("thread %s panicked at %s:%s\n", Thread.currentThread().getName(), stackTop.getFileName(), stackTop.getLineNumber());
        out.println(e.getMessage());

        switch (backtraceLevel) {
            case BacktraceLevel.None -> {
                out.println("note: run with `JAVA_BACKTRACE=1` environment variable to display a backtrace");
            }
            case BacktraceLevel.Minimal -> {
                out.println("stack trace:");

                for (int i = 0; i < MINIMAL_ITERATIONS; i++) {
                    StackTraceElement trace = stack[i];
                    out.printf("%d: %s::%s\n", i, trace.getClassName(), trace.getMethodName());
                    out.printf("\tat %s:%d\n", trace.getFileName(), trace.getLineNumber());
                }
            }
            case BacktraceLevel.Full -> {
                out.println("stack trace:");

                for (int i = 0; i < stack.length; i++) {
                    StackTraceElement trace = stack[i];
                    out.printf("%d: %s::%s\n", i, trace.getClassName(), trace.getMethodName());
                    out.printf("\tat %s:%d\n", trace.getFileName(), trace.getLineNumber());
                }
            }
        }
    }

    private enum BacktraceLevel {
        None,
        Minimal,
        Full
    }
}
