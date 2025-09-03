package frc.robot.subsystems.ground_intake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs { }

    public default void periodic() {}
    public default void updateInputs(GroundIntakeIOInputs inputs) { }

    public default void deploy() {}
    public default void retract() {}

    public default void startRoller() {}
    public default void stopRoller() {}
}
