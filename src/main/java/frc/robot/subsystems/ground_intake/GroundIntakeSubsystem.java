// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ground_intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final GroundIntakeIO m_io;
    private final GroundIntakeIOInputsAutoLogged m_inputs = new GroundIntakeIOInputsAutoLogged();

    public GroundIntakeSubsystem(GroundIntakeIO io) {
        m_io = io;

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    @Override
    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
    }

    public Command retract() {
        return OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.HOME)
            .andThen(runOnce(() -> {
                m_io.stopRoller();
                m_io.retract();
            }));
    }

    private void scoreMechanismStateChangeCallback() {
        if (OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE) {
            m_io.startRoller();
            m_io.deploy();
        }
    }
}
