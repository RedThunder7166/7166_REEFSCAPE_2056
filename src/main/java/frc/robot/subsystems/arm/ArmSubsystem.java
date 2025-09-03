// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO m_io;
    private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        m_io = io;

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    @Override
    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
    }

    private void scoreMechanismStateChangeCallback() {
        switch (OurRobotState.getScoreMechanismState()) {
            case HOME:
                m_io.pivotGoPosition(pivotPositionGrab);
                break;
            case CORAL_INTAKE:
            // case CORAL_LOADED:
                break;

            case CORAL_SCORE_L1:
            case CORAL_SCORING_L1:
                m_io.pivotGoPosition(pivotPositionL1Coral);
                break;
            case CORAL_SCORE_L2:
            case CORAL_SCORING_L2:
                m_io.pivotGoPosition(pivotPositionL2Coral);
                break;
            case CORAL_SCORE_L3:
            case CORAL_SCORING_L3:
                m_io.pivotGoPosition(pivotPositionL3Coral);
                break;
            case CORAL_SCORE_L4:
            case CORAL_SCORING_L4:
                m_io.pivotGoPosition(pivotPositionL4Coral);
                break;

            case ALGAE_PICKUP_FLOOR:
                m_io.pivotGoPosition(pivotPositionAlgaePickupFloor);
                break;
            case ALGAE_PICKUP_L2:
                m_io.pivotGoPosition(pivotPositionAlgaePickupL2);
                break;
            case ALGAE_PICKUP_L3:
                m_io.pivotGoPosition(pivotPositionAlgaePickupL3);
                break;

            case ALGAE_HOME:
                m_io.pivotGoPosition(pivotPositionAlgaeHome);
                break;

            case ALGAE_SCORE_NET:
            case ALGAE_SCORING_NET:
                m_io.pivotGoPosition(pivotPositionNet);
                break;
            case ALGAE_SCORE_PROCESSOR:
            case ALGAE_SCORING_PROCESSOR:
                m_io.pivotGoPosition(pivotPositionProcessor);
                break;
        }
    }

    public Command gripperCoralOn() {
        return runOnce(m_io::gripperCoralOn);
    }
    public Command gripperAlgaeOn() {
        return runOnce(m_io::gripperAlgaeOn);
    }

    public Command gripperOff() {
        return runOnce(m_io::gripperOff);
    }
}
