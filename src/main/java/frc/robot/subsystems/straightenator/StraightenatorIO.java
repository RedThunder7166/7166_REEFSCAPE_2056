package frc.robot.subsystems.straightenator;

import org.littletonrobotics.junction.AutoLog;

public interface StraightenatorIO {
    @AutoLog
    public static class StraightenatorIOInputs {
        double leftMotorVelocityRPS;
        double leftMotorCurrentAmps;

        double rightMotorVelocityRPS;
        double rightMotorCurrentAmps;
    }

    public default void periodic() { }
    public default void updateInputs(StraightenatorIOInputs inputs) { }

    public default void on() { }
    public default void off() { }
}
