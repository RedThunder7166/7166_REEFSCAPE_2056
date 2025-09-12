// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;
import frc.robot.util.GeneralUtils;
import frc.robot.util.OptionalDistanceSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();
    private OptionalDistanceSupplier m_manualTargetDistanceSupplier = null;

    public ElevatorSubsystem(ElevatorIO io) {
        m_io = io;

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    public void setManualTargetDistanceSupplier(OptionalDistanceSupplier supplier) {
        m_manualTargetDistanceSupplier = supplier;
    }

    @Override
    public void periodic() {
        if (m_manualTargetDistanceSupplier != null) {
            m_manualTargetDistanceSupplier.getAsOptionalDistance().ifPresent((Distance distance) -> {
                goDistance(distance).schedule();
            });
        }

        m_io.updateInputs(m_inputs);
        OurRobotState.setIsElevatorAboveArmFarNetClearance(m_io.isAtOrAbovePosition(positionArmFarNetClearance));
        OurRobotState.setIsElevatorAboveArmCoralHolderClearance(m_io.isAtOrAbovePosition(positionArmCoralHolderClearance));

        Logger.processInputs("ElevatorSubsystem", m_inputs);
    }

    public Command goPosition(double position) {
        final Command waitCommand =
            // position <= positionArmCoralHolderClearance ? Commands.waitUntil(GeneralUtils.negateBooleanSupplier(OurRobotState::getIsArmPastCoralHolderClearance)) :
            position <= positionArmCoralHolderClearance ? Commands.waitUntil(GeneralUtils.negateBooleanSupplier(OurRobotState::getIsArmWithinCoralHolderRange)) :
            position <= positionArmFarNetClearance ? Commands.waitUntil(GeneralUtils.negateBooleanSupplier(OurRobotState::getIsArmPastFarNetClearance)) :
            Commands.none();
        // final Command waitCommand = Commands.none();
        return waitCommand.andThen(runOnce(() -> m_io.goPosition(position))
            .until(() -> m_io.isAtPosition(position)));
    }
    public Command goDistance(Distance distance) {
        return goPosition(distanceToMechanismPosition(distance));
    }

    private void scoreMechanismStateChangeCallback() {
        switch (OurRobotState.getScoreMechanismState()) {
            case HOME:
                goPosition(positionHome).schedule();
                break;
            case CORAL_INTAKE:
            // case CORAL_LOADED:
                break;

            case CORAL_SCORE_L1:
                goPosition(positionL1Coral).schedule();
                break;
            case CORAL_SCORING_L1:
                goPosition(positionL1CoralScore).schedule();
                break;
            case CORAL_SCORE_L2:
                goPosition(positionL2Coral).schedule();
                break;
            case CORAL_SCORING_L2:
                goPosition(positionL2CoralScore).schedule();
                break;
            case CORAL_SCORE_L3:
                goPosition(positionL3Coral).schedule();
                break;
            case CORAL_SCORING_L3:
                goPosition(positionL3CoralScore).schedule();
                break;
            case CORAL_SCORE_L4:
                goPosition(positionL4Coral).schedule();
                break;
            case CORAL_SCORING_L4:
                goPosition(positionL4CoralScore).schedule();
                break;

            case ALGAE_PICKUP_FLOOR:
                goPosition(positionFloorAlgae).schedule();
                break;
            case ALGAE_PICKUP_L2:
                goPosition(positionL2Algae).schedule();
                break;
            case ALGAE_PICKUP_L3:
                goPosition(positionL3Algae).schedule();
                break;
            case ALGAE_HOME:
                goPosition(positionAlgaeHome).schedule();
                break;
            case ALGAE_SCORE_NET:
                goPosition(positionNet).schedule();
                break;
            case ALGAE_SCORE_PROCESSOR:
                goPosition(positionProcessor).schedule();
                break;

            case ALGAE_SCORING_NET:
            case ALGAE_SCORING_PROCESSOR:
                break;
        }
    }
}
