package frc.robot.subsystems.straightenator;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class StraightenatorConstants {
    public static final boolean ENABLE = false; // MOTORS WILL NOT SPIN IF THIS IS FALSE

    public static final double motorReduction = 0d;
    public static final int currentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public static final int firstSensorChannel = -1;
    public static final int secondSensorChannel = -1;
    public static final double intakeOutput = 0.15;

    // POSITIVE INTAKES IN (TOWARD THE HOLDER)
    public static final int leftMotorId = 19;
    public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;

    // POSITIVE INTAKES IN (TOWARD THE HOLDER)
    public static final int rightMotorId = 20;
    public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;
}
