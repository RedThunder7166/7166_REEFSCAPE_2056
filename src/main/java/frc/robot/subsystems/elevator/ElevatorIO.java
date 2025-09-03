package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs { }

    public default void updateInputs(ElevatorIOInputs inputs) { }

    public default void neutral() {}
    public default void goPosition(double position) { }

    public default boolean isAtPosition(double position) { return false; }
}
