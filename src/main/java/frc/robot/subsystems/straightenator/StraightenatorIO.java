package frc.robot.subsystems.straightenator;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.BeamBreakSensor;

public interface StraightenatorIO {
    @AutoLog
    public static class StraightenatorIOInputs {
        double leftMotorVelocityRPS;
        double leftMotorCurrentAmps;

        double rightMotorVelocityRPS;
        double rightMotorCurrentAmps;

        boolean firstSensorTripped;
        boolean secondSensorTripped;
    }

    public default StraightenatorIO withFirstSensor(BeamBreakSensor sensor) { return this; }
    public default StraightenatorIO withSecondSensor(BeamBreakSensor sensor) { return this; }

    public default void periodic() { }
    public default void updateInputs(StraightenatorIOInputs inputs) { }

    public default void on() { }
    public default void off() { }
}
