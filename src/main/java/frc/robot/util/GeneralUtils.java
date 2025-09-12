package frc.robot.util;

import java.util.function.BooleanSupplier;

public class GeneralUtils {
    public static BooleanSupplier negateBooleanSupplier(BooleanSupplier supplier) {
        return () -> !supplier.getAsBoolean();
    }
}
