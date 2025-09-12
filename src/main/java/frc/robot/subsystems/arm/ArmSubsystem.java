// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;
import frc.robot.util.BeamBreakSensor;
import frc.robot.util.GeneralUtils;
import frc.robot.util.OptionalAngleSupplier;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO m_io;
    private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
    public final BeamBreakSensor m_gripperSensor = new BeamBreakSensor(gripperSensorId, null);
    private OptionalAngleSupplier m_manualPivotAngleSupplier = null;

    public ArmSubsystem(ArmIO io) {
        m_io = io.withGripperSensor(m_gripperSensor);

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    public void setManualPivotAngleSupplier(OptionalAngleSupplier supplier) {
        m_manualPivotAngleSupplier = supplier;
    }

    private boolean temp = false;
    @Override
    public void periodic() {
        // temp = false;
        if (m_manualPivotAngleSupplier != null) {
            m_manualPivotAngleSupplier.getAsOptionalAngle().ifPresent((Angle angle) -> {
                pivotGoAngle(angle).schedule();
                // temp = true;
            });
        }

        // if (!temp) {
        //     if (OurRobotState.getScoreMechanismState() == ScoreMechanismState.HOME)
        //         pivotGoPosition(pivotPositionGrab).schedule();
        // }

        m_io.periodic();
        m_io.updateInputs(m_inputs);

        OurRobotState.setIsArmPastFarNetClearance(m_io.pivotIsAtOrPastPosition(pivotPositionFarNetClearance));
        OurRobotState.setIsArmPastCoralHolderClearance(m_io.pivotIsAtOrPastPosition(pivotPositionCoralHolderClearance));
        OurRobotState.setIsArmWithinCoralHolderRange(m_io.pivotIsAtOrBeforePosition(pivotPositionCoralHolderClearanceHigh) && m_io.pivotIsAtOrPastPosition(pivotPositionCoralHolderClearanceLow));

        // TODO: we probably only set this here when it's true; then, when we score a piece (press score button / set state / etc) it becomes false
        OurRobotState.setIsCoralInGripper(m_inputs.gripperSensorTripped);

        Logger.processInputs("ArmSubsystem", m_inputs);
    }

    public Command pivotGoPosition(double position) {
        final Command waitCommand =
            position >= pivotPositionFarNetClearance ? Commands.waitUntil(OurRobotState::getIsElevatorAboveArmFarNetClearance) :
            // position <= pivotPositionGrab ? Commands.waitUntil(OurRobotState::getIsElevatorAboveArmCoralHolderClearance) :
            // // Commands.waitUntil(GeneralUtils.negateBooleanSupplier(OurRobotState::getIsElevatorAboveArmFarNetClearance));
            (position <= pivotPositionCoralHolderClearanceHigh && position >= pivotPositionCoralHolderClearanceLow) ? Commands.waitUntil(OurRobotState::getIsElevatorAboveArmCoralHolderClearance) :
            Commands.none();
        // final Command waitCommand = Commands.none();
        return waitCommand.andThen(runOnce(() -> m_io.pivotGoPosition(position))
            .until(() -> m_io.pivotIsAtPosition(position)));
    }
    public Command pivotGoAngle(Angle angle) {
        return pivotGoPosition(pivotAngleToMechanismPosition(angle));
    }

    public Command gripperCoralOn() {
        return runOnce(m_io::gripperCoralOn);
    }
    public Command gripperAlgaeOn() {
        return runOnce(m_io::gripperAlgaeOn);
    }

    public Command gripperReverse() {
        return runOnce(m_io::gripperReverse);
    }
    public Command gripperOff() {
        return runOnce(m_io::gripperOff);
    }

    private Command score() {
        return runOnce(() -> {
            m_io.gripperReverse();
        });
    }

    private void scoreMechanismStateChangeCallback() {
        switch (OurRobotState.getScoreMechanismState()) {
            case HOME:
                gripperOff()
                    .andThen(pivotGoPosition(pivotPositionGrab))
                    .schedule();
                break;
            case CORAL_INTAKE:
            // case CORAL_LOADED:
                gripperOff().schedule();
                break;

            case CORAL_SCORE_L1:
                pivotGoPosition(pivotPositionL1Coral).schedule();
                break;
            case CORAL_SCORING_L1:
                pivotGoPosition(pivotPositionL1CoralScore)
                    .andThen(score())
                    .schedule();
                break;

            case CORAL_SCORE_L2:
                pivotGoPosition(pivotPositionL2Coral).schedule();
                break;
            case CORAL_SCORING_L2:
                pivotGoPosition(pivotPositionL2CoralScore)
                    .andThen(score())
                    .schedule();
                break;

            case CORAL_SCORE_L3:
                pivotGoPosition(pivotPositionL3Coral).schedule();
                break;
            case CORAL_SCORING_L3:
                pivotGoPosition(pivotPositionL3CoralScore)
                    .andThen(score())
                    .schedule();
                break;

            case CORAL_SCORE_L4:
                pivotGoPosition(pivotPositionL4Coral).schedule();
                break;
            case CORAL_SCORING_L4:
                pivotGoPosition(pivotPositionL4CoralScore)
                    .andThen(score())
                    .schedule();
                break;

            case ALGAE_PICKUP_FLOOR:
                pivotGoPosition(pivotPositionAlgaePickupFloor)
                    .andThen(gripperAlgaeOn())
                    .schedule();
                break;
            case ALGAE_PICKUP_L2:
                pivotGoPosition(pivotPositionAlgaePickupL2)
                    .andThen(gripperAlgaeOn())
                    .schedule();
                break;
            case ALGAE_PICKUP_L3:
                pivotGoPosition(pivotPositionAlgaePickupL3)
                    .andThen(gripperAlgaeOn())
                    .schedule();
                break;

            case ALGAE_HOME:
                pivotGoPosition(pivotPositionAlgaeHome).schedule();
                break;

            case ALGAE_SCORE_NET:
                pivotGoPosition(pivotPositionNet).schedule();
                break;
            case ALGAE_SCORING_NET:
                gripperReverse()
                    .andThen(pivotGoPosition(pivotPositionNet))
                    .schedule();
                break;

            case ALGAE_SCORE_PROCESSOR:
                pivotGoPosition(pivotPositionProcessor).schedule();
                break;
            case ALGAE_SCORING_PROCESSOR:
                gripperReverse()
                    .andThen(pivotGoPosition(pivotPositionProcessor))
                    .schedule();
                break;
        }
    }
}
