package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final double motorReduction = 5d; // motor spins x times for mechanism to "spin" once
    public static final double pitchCircumferenceMM = 150;
    public static final int currentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    // FIXME: elevator is at position threshold
    public static final double isAtPositionThreshold = 0.02d;

    public static final int leaderMotorId = 13; // left (by the CANivore)
    public static final InvertedValue leaderInverted = InvertedValue.Clockwise_Positive;

    public static final int followerMotorId = 6; // right
    public static final InvertedValue followerInverted = InvertedValue.CounterClockwise_Positive;

    public static double distanceToMechanismPosition(Distance distance) {
        return distance.in(Millimeters) / pitchCircumferenceMM;
    }
    public static Distance mechanismPositionToDistance(double position) {
        return Millimeters.of(position * pitchCircumferenceMM);
    }

    public static final double positionMAX = 8.5;
    public static final double positionMIN = 0;

    public static final double positionHome = distanceToMechanismPosition(Inches.of(2d)); // 1.88
    // public static final double positionGrab = distanceToMechanismPosition(Inches.of(0));
    public static final double positionGrab = positionMIN;
    public static final double positionInitial = positionGrab;
    public static final double positionArmCoralHolderClearance = distanceToMechanismPosition(Inches.of(3.065)); // 2.954:0.519

    // public static final double positionL1Coral = distanceToMechanismPosition(Inches.of(0d));
    public static final double positionL1Coral = positionArmCoralHolderClearance + 0.1;
    public static final double positionL1CoralScore = positionL1Coral;
    public static final double positionL2Coral = positionL1Coral;
    public static final double positionL2CoralScore = positionL2Coral;
    public static final double positionL3Coral = distanceToMechanismPosition(Inches.of(16d)); // 2.737
    public static final double positionL3CoralScore = positionL3Coral;
    // public static final double positionL4Coral = distanceToMechanismPosition(Inches.of(39.9d)); //6.77d
    // public static final double positionL4CoralScore = positionL4Coral;
    public static final double positionL4Coral = positionMAX;
    // public static final double positionL4CoralScore = positionMAX - distanceToMechanismPosition(Inches.of(5d));//7.64
    // public static final double positionL4CoralScore = distanceToMechanismPosition(Inches.of(43.4d));
    public static final double positionL4CoralScore = positionL4Coral;

    public static final double positionProcessor = distanceToMechanismPosition(Inches.of(0));
    public static final double positionNet = distanceToMechanismPosition(Inches.of(0));

    public static final double positionFloorAlgae = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL2Algae = distanceToMechanismPosition(Inches.of(11.5d)); // 11.5
    public static final double positionL3Algae = distanceToMechanismPosition(Inches.of(26.5d)); // 26.5

    private static final double positionArmFarNetClearanceInches = 22.7d; // 22.57:3.844
    public static final double positionArmFarNetClearance = distanceToMechanismPosition(Inches.of(positionArmFarNetClearanceInches));

    public static final double positionAlgaeHome = distanceToMechanismPosition(Inches.of(positionArmFarNetClearanceInches + 2d));

    public static final double pidP = 16;
    public static final double cruiseVelocity = 14;
    public static final double acceleration = 30;
}
