package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs { }

    public default void periodic() { }
    public default void updateInputs(ArmIOInputs inputs) { }

    public default void pivotGoPosition(double position) { }

    public default boolean pivotIsAtPosition(double position) { return false; }

    public default void gripperCoralOn() { }
    public default void gripperAlgaeOn() { }
    public default void gripperOff() { }
}
