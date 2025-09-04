package frc.robot.subsystems.ground_intake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        double targetActuatorMotorPositionRotations;

        double rollerMotorVelocityRPS;
        double rollerMotorCurrentAmps;

        double actuatorMotorPositionRotations;
        double actuatorMotorCurrentAmps;
    }

    public default void periodic() {}
    public default void updateInputs(GroundIntakeIOInputs inputs) { }

    public default void deploy() {}
    public default void retract() {}

    public default boolean actuatorIsAtPosition(double position) { return false; }

    public default void startRoller() {}
    public default void stopRoller() {}
}
