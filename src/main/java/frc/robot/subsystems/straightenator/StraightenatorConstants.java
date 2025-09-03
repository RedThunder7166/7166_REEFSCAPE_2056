package frc.robot.subsystems.straightenator;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class StraightenatorConstants {
    public static final double motorReduction = 0d;
    public static final int currentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    public static final int firstSensorId = -1;
    public static final int secondSensorId = -1;
    public static final double intakeOutput = 0.1;

    // POSITIVE INTAKES IN
    public static final int leftMotorId = -1;
    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;

    // POSITIVE INTAKES IN
    public static final int rightMotorId = -1;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;
}
