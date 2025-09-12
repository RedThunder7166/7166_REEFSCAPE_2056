package frc.robot.subsystems.straightenator;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class StraightenatorConstants {
    public static final boolean ENABLE = true; // MOTORS WILL NOT SPIN IF THIS IS FALSE

    public static final double motorReduction = 0d;
    public static final int currentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public static final int firstSensorChannel = 3;
    public static final int secondSensorChannel = 2;
    public static final double intakeOutput = 0.32d;
    // public static final double antiJamReverseOutput = -(intakeOutput / 2d);
    public static final double antiJamReverseOutput = -intakeOutput;

    // POSITIVE INTAKES IN (TOWARD THE HOLDER)
    public static final int leftMotorId = 19;
    public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;

    // POSITIVE INTAKES IN (TOWARD THE HOLDER)
    public static final int rightMotorId = 20;
    public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;

    public static final double antiJamCurrentThreshold = 7d;
    public static final double antiJamDurationSeconds = 1d;
}
