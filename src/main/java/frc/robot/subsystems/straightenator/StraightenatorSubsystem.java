// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;

public class StraightenatorSubsystem extends SubsystemBase {
    private final StraightenatorIO m_io;
    private final StraightenatorIOInputsAutoLogged m_inputs = new StraightenatorIOInputsAutoLogged();

    public StraightenatorSubsystem(StraightenatorIO io) {
        m_io = io;

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    @Override
    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
    }

    public Command on() {
        return runOnce(() -> m_io.on());
    }

    private void scoreMechanismStateChangeCallback() {
        if (OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE)
            on().schedule();
    }
}
