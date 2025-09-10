package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;

@FunctionalInterface
public interface OptionalDistanceSupplier {
    Optional<Distance> getAsOptionalDistance();
}