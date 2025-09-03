package frc.robot.subsystems.ground_intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GroundIntakeConstants {
    public static final double rollerMotorReduction = 0d;
    public static final int rollerCurrentLimit = 40;
    public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Brake;

    public static final double actuatorMotorReduction = 0d;
    public static final int actuatorCurrentLimit = 40;
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;

    public static final int beamBreakId = -1;

    // POSITIVE INTAKES IN
    public static final int rollerMotorId = -1;
    public static final InvertedValue rollerInverted = InvertedValue.CounterClockwise_Positive;
    public static final double rollerOutput = 1d;

    // POSITIVE IS OUT (LIKE WHEN DEPLOYING)
    public static final int actuatorMotorId = -1;
    public static final InvertedValue actuatorInverted = InvertedValue.Clockwise_Positive;

    public static final double actuatorPidP = 1;
    public static final double actuatorCruiseVelocity = 1;
    public static final double actuatorAcceleration = 1;

    public static final double actuatorPositionHome = 0d;
    public static final double actuatorPositionDeployed = 0d;
}
