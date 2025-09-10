package frc.robot.subsystems.ground_intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.BeamBreakSensor;

public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        double targetActuatorMotorPositionRotations;

        double rollerMotorVelocityRPS;
        double rollerMotorCurrentAmps;

        double actuatorMotorPositionRotations;
        double actuatorMotorPositionDegrees;
        double actuatorMotorCurrentAmps;

        boolean beamBreakTripped;
    }

    public default GroundIntakeIO withBeamBreak(BeamBreakSensor sensor) { return this; }

    public default void periodic() {}
    public default void updateInputs(GroundIntakeIOInputs inputs) { }

    public default void deploy() {}
    public default void retract() {}

    public default boolean actuatorIsAtPosition(double position) { return false; }

    public default void startRoller() {}
    public default void stopRoller() {}
}
