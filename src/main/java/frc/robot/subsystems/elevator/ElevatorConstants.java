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
    public static final double isAtPositionThreshold = 0.001d;

    public static final int leaderMotorId = -1; // right
    public static final InvertedValue leaderInverted = InvertedValue.Clockwise_Positive;

    public static final int followerMotorId = -1; // left (by the CANivore)
    public static final InvertedValue followerInverted = InvertedValue.CounterClockwise_Positive;

    public static double distanceToMechanismPosition(Distance distance) {
        return distance.in(Millimeters) / pitchCircumferenceMM;
    }

    public static final double positionHome = distanceToMechanismPosition(Inches.of(0));
    public static final double positionGrab = distanceToMechanismPosition(Inches.of(0));
    public static final double positionInitial = positionGrab;
    public static final double positionArmClearance = positionHome - distanceToMechanismPosition(Inches.of(0.25));

    public static final double positionL1Coral = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL1CoralDropped = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL2Coral = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL2CoralDropped = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL3Coral = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL3CoralDropped = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL4Coral = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL4CoralDropped = distanceToMechanismPosition(Inches.of(0));

    public static final double positionProcessor = distanceToMechanismPosition(Inches.of(0));
    public static final double positionNet = distanceToMechanismPosition(Inches.of(0));

    public static final double positionFloorAlgae = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL2Algae = distanceToMechanismPosition(Inches.of(0));
    public static final double positionL3Algae = distanceToMechanismPosition(Inches.of(0));
    public static final double positionAlgaeHome = distanceToMechanismPosition(Inches.of(0));

    // arm degree > 90
    public static final double positionArmFarNetClearance = distanceToMechanismPosition(Inches.of(30));

    public static final double pidP = 1;
    public static final double cruiseVelocity = 1;
    public static final double acceleration = 1;
}
