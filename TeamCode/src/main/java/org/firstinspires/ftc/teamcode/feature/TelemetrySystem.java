package org.firstinspires.ftc.teamcode.feature;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class TelemetrySystem {
    private static Telemetry telemetry;
    private static boolean isInitialized = false;
    private static Map<String, Boolean> enabledClasses = new HashMap<>();
    private static boolean isDebugMode = true;

    private static void checkInitialization() {
        if (!isInitialized) {
            throw new IllegalStateException("TelemetrySystem not initialized. Call init() first.");
        }
    }

    public static void init(Telemetry telemetry) {
        if (isInitialized) return;
        TelemetrySystem.telemetry = telemetry;
        isInitialized = true;
    }

    public static Telemetry getTelemetry() {
        checkInitialization();
        return telemetry;
    }

    public static void addClassData(String className, String caption, Object value) {
        checkInitialization();
        if (!enabledClasses.containsKey(className) || Boolean.TRUE.equals(enabledClasses.get(className))) {
            telemetry.addData("[" + className + "] " + caption, value);
        }
    }

    public static void addDebugData(String caption, Object value) {
        if (isDebugMode) {
            checkInitialization();
            telemetry.addData("[DEBUG] " + caption, value);
        }
    }

    public static void enableClass(String className) {
        checkInitialization();
        enabledClasses.put(className, true);
    }

    public static void disableClass(String className) {
        checkInitialization();
        enabledClasses.put(className, false);
    }

    public static void setDebugMode(boolean isDebugMode) {
        checkInitialization();
        TelemetrySystem.isDebugMode = isDebugMode;
    }

    public static void update() {
        checkInitialization();
        telemetry.update();
    }
}
