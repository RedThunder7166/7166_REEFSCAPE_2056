package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;

@FunctionalInterface
public interface OptionalAngleSupplier {
    Optional<Angle> getAsOptionalAngle();
}