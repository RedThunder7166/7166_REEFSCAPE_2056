package frc.robot.subsystems.straightenator;

import org.littletonrobotics.junction.AutoLog;

public interface StraightenatorIO {
    @AutoLog
    public static class StraightenatorIOInputs { }

    public default void periodic() { }
    public default void updateInputs(StraightenatorIOInputs inputs) { }

    public default void on() { }
    public default void off() { }
}
