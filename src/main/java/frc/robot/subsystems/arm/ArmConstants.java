package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.arm.ArmConstants.pivotPositionGrab;
import static frc.robot.subsystems.elevator.ElevatorConstants.positionL4Coral;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
    public static final double pivotMotorReduction = 54.95d;
    public static final int pivotCurrentLimit = 40;
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;
    // FIXME: arm pivot is at position threshold
    // public static final double pivotIsAtPositionThreshold = 0.02d;
    public static final double pivotIsAtPositionThreshold = 0.014d;

    public static final int gripperCurrentLimit = 40;
    public static final NeutralModeValue gripperNeutralMode = NeutralModeValue.Brake;
    public static final double gripperCoralOutput = 0.4d;
    public static final double gripperAlgaeOutput = 0.3d;

    public static final int gripperSensorId = 0;

    // POSITIVE IS UP (CLOCKWISE MOTION)
    public static final int pivotMotorId = 18;
    public static final InvertedValue pivotInverted = InvertedValue.CounterClockwise_Positive;

    // POSITIVE IS TOWARD THE ROBOT (INTAKE)
    public static final int gripperMotorId = 11;
    public static final InvertedValue gripperInverted = InvertedValue.Clockwise_Positive;


    public static double pivotAngleToMechanismPosition(Angle angle) {
        return angle.in(Degrees) / 360d;
    }
    public static final Angle mechanismPositionToPivotAngle(double position) {
        return Degrees.of(position * 360d);
    }

    public static final double positionMAX = pivotAngleToMechanismPosition(Degrees.of(195d));

    public static final double pivotPositionGrab = pivotAngleToMechanismPosition(Degrees.of(-81d));
    public static final double pivotPositionCoralHolderClearanceHigh = pivotAngleToMechanismPosition(Degrees.of(-52d));
    // public static final double pivotPositionCoralHolderClearanceLow = pivotAngleToMechanismPosition(Degrees.of(-76d));
    // public static final double pivotPositionCoralHolderClearanceLow = pivotPositionGrab;
    public static final double pivotPositionCoralHolderClearanceLow = pivotAngleToMechanismPosition(Degrees.of(-83d));
    public static final double pivotPositionCoralHolderClearance = pivotAngleToMechanismPosition(Degrees.of(-46d));
    public static final double pivotPositionInitial = pivotPositionGrab;

    public static final double positionMIN = pivotPositionGrab;

    public static final double pivotPositionL1Coral = pivotAngleToMechanismPosition(Degrees.of(47.19d)); // 47.19:0.1478
    public static final double pivotPositionL1CoralScore = pivotAngleToMechanismPosition(Degrees.of(34.71d)); //41.0; 34.71
    public static final double pivotPositionL2Coral = pivotPositionL1Coral;
    public static final double pivotPositionL2CoralScore = pivotPositionL1CoralScore;
    public static final double pivotPositionL3Coral = pivotPositionL1Coral;
    public static final double pivotPositionL3CoralScore = pivotPositionL1CoralScore;
    public static final double pivotPositionL4Coral = pivotAngleToMechanismPosition(Degrees.of(26d));
    // public static final double pivotPositionL4CoralScore = pivotPositionL1CoralScore;
    public static final double pivotPositionL4CoralScore = pivotAngleToMechanismPosition(Degrees.of(5d));

    public static final double pivotPositionAlgaePickupFloor = pivotPositionGrab;
    public static final double pivotPositionAlgaePickupL2 = pivotAngleToMechanismPosition(Degrees.of(0d));
    public static final double pivotPositionAlgaePickupL3 = pivotAngleToMechanismPosition(Degrees.of(0d));

    public static final double pivotPositionAlgaeHome = pivotAngleToMechanismPosition(Degrees.of(90d));

    public static final double pivotPositionNet = pivotPositionGrab;
    public static final double pivotPositionProcessor = pivotPositionGrab;
    // the elevator should not go down when the arm pivot degree >= this point
    public static final double pivotPositionFarNetClearance = pivotAngleToMechanismPosition(Degrees.of(82.944d)); // 88.15:0.2304
    // for when we target net on the opposite side of the robot front
    public static final double pivotPositionFarNet = pivotPositionFarNetClearance;

    public static final double pidP = 40;
    public static final double cruiseVelocity = 10;
    public static final double acceleration = 20;
}
