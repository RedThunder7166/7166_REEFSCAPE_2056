package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        double targetMotorPositionRotations;

        double leadMotorPositionRotations;
        double leadMotorCurrentAmps;

        double followerMotorPositionRotations;
        double followerMotorCurrentAmps;
    }

    public default void updateInputs(ElevatorIOInputs inputs) { }
    public default void periodic() { }

    public default void neutral() {}
    public default void goPosition(double position) { }

    public default boolean isAtPosition(double position) { return false; }
    public default boolean isAtOrAbovePosition(double position) { return false; }
}
