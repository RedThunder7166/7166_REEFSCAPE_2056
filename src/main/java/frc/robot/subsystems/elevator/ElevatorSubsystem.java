// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        m_io = io;

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
    }

    public Command goPosition(double position) {
        return runOnce(() -> m_io.goPosition(position))
            .until(() -> m_io.isAtPosition(position));
    }

    private void scoreMechanismStateChangeCallback() {
        switch (OurRobotState.getScoreMechanismState()) {
            case HOME:
                m_io.goPosition(positionHome);
                break;
            case CORAL_INTAKE:
            // case CORAL_LOADED:
                break;

            case CORAL_SCORE_L1:
                m_io.goPosition(positionL1Coral);
                break;
            case CORAL_SCORING_L1:
                m_io.goPosition(positionL1CoralDropped);
                break;
            case CORAL_SCORE_L2:
                m_io.goPosition(positionL2Coral);
                break;
            case CORAL_SCORING_L2:
                m_io.goPosition(positionL2CoralDropped);
                break;
            case CORAL_SCORE_L3:
                m_io.goPosition(positionL3Coral);
                break;
            case CORAL_SCORING_L3:
                m_io.goPosition(positionL3CoralDropped);
                break;
            case CORAL_SCORE_L4:
                m_io.goPosition(positionL4Coral);
                break;
            case CORAL_SCORING_L4:
                m_io.goPosition(positionL4CoralDropped);
                break;

            case ALGAE_PICKUP_FLOOR:
                m_io.goPosition(positionFloorAlgae);
                break;
            case ALGAE_PICKUP_L2:
                m_io.goPosition(positionL2Algae);
                break;
            case ALGAE_PICKUP_L3:
                m_io.goPosition(positionL3Algae);
                break;
            case ALGAE_HOME:
                m_io.goPosition(positionAlgaeHome);
                break;
            case ALGAE_SCORE_NET:
                m_io.goPosition(positionNet);
                break;
            case ALGAE_SCORE_PROCESSOR:
                m_io.goPosition(positionProcessor);
                break;

            case ALGAE_SCORING_NET:
            case ALGAE_SCORING_PROCESSOR:
                break;
        }
    }
}
