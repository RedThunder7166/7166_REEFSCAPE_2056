package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public static final double pivotMotorReduction = 54.95d;
    public static final int pivotCurrentLimit = 40;
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;
    // FIXME: arm pivot is at position threshold
    public static final double pivotIsAtPositionThreshold = 0.001d;

    public static final int gripperCurrentLimit = 40;
    public static final NeutralModeValue gripperNeutralMode = NeutralModeValue.Brake;
    public static final double gripperCoralOutput = 0.1d;
    public static final double gripperAlgaeOutput = 0.1d;
    public static final double gripperReverseOutput = -0.2d;

    public static final int gripperSensorId = -1;

    // POSITIVE IS UP (CLOCKWISE MOTION)
    public static final int pivotMotorId = 18;
    public static final InvertedValue pivotInverted = InvertedValue.Clockwise_Positive;

    // POSITIVE IS TOWARD THE ROBOT (INTAKE)
    public static final int gripperMotorId = 11;
    public static final InvertedValue gripperInverted = InvertedValue.CounterClockwise_Positive;


    public static double pivotAngleToMechanismPosition(Angle angle) {
        return angle.in(Degrees) / 360d;
    }
    public static final Angle mechanismPositionToPivotAngle(double position) {
        return Degrees.of(position * 360d);
    }

    public static final double pivotPositionGrab = pivotAngleToMechanismPosition(Degrees.of(-81));
    public static final double pivotPositionInitial = pivotPositionGrab;

    public static final double pivotPositionL1Coral = pivotPositionGrab;
    public static final double pivotPositionL2Coral = pivotPositionGrab;
    public static final double pivotPositionL3Coral = pivotPositionGrab;
    public static final double pivotPositionL4Coral = pivotPositionGrab;

    public static final double pivotPositionAlgaePickupFloor = pivotPositionGrab;
    public static final double pivotPositionAlgaePickupL2 = pivotPositionGrab;
    public static final double pivotPositionAlgaePickupL3 = pivotPositionGrab;

    public static final double pivotPositionAlgaeHome = pivotPositionGrab;

    public static final double pivotPositionNet = pivotPositionGrab;
    public static final double pivotPositionProcessor = pivotPositionGrab;
    // the elevator should not go down when the arm pivot degree >= this point
    public static final double pivotPositionFarNetClearance = pivotAngleToMechanismPosition(Degrees.of(90));
    // for when we target net on the opposite side of the robot front
    public static final double pivotPositionFarNet = pivotPositionGrab;

    public static final double pidP = 1;
    public static final double cruiseVelocity = 1;
    public static final double acceleration = 1;
}
