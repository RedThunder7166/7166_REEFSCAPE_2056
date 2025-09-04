package frc.robot.subsystems.ground_intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class GroundIntakeConstants {
    public static final double rollerMotorReduction = 0d;
    public static final int rollerCurrentLimit = 40;
    public static final NeutralModeValue rollerNeutralMode = NeutralModeValue.Brake;

    public static final double actuatorMotorReduction = 56.12d;
    public static final int actuatorCurrentLimit = 40;
    public static final NeutralModeValue actuatorNeutralMode = NeutralModeValue.Brake;
    // FIXME: arm pivot is at position threshold
    public static final double actuatorIsAtPositionThreshold = 0.001d;

    public static final int beamBreakId = -1;

    // POSITIVE INTAKES IN
    public static final int rollerMotorId = -1;
    public static final InvertedValue rollerInverted = InvertedValue.CounterClockwise_Positive;
    public static final double rollerOutput = 1d;

    // POSITIVE IS OUT (LIKE WHEN DEPLOYING)
    public static final int actuatorMotorId = -1;
    public static final InvertedValue actuatorInverted = InvertedValue.Clockwise_Positive;

    public static double actuatorAngleToMechanismPosition(Angle angle) {
        return angle.in(Degrees) / 360d;
    }

    public static final double actuatorPositionHome = actuatorAngleToMechanismPosition(Degrees.of(112d));
    public static final double actuatorPositionDeployed = actuatorAngleToMechanismPosition(Degrees.of(-35d));

    public static final double actuatorPidP = 1;
    public static final double actuatorCruiseVelocity = 1;
    public static final double actuatorAcceleration = 1;
}
