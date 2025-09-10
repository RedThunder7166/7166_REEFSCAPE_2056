// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import static frc.robot.subsystems.straightenator.StraightenatorConstants.firstSensorChannel;
import static frc.robot.subsystems.straightenator.StraightenatorConstants.secondSensorChannel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;
import frc.robot.util.BeamBreakSensor;

public class StraightenatorSubsystem extends SubsystemBase {
    private final StraightenatorIO m_io;
    private final StraightenatorIOInputsAutoLogged m_inputs = new StraightenatorIOInputsAutoLogged();
    public final BeamBreakSensor m_firstSensor = new BeamBreakSensor(firstSensorChannel, null);
    public final BeamBreakSensor m_secondSensor = new BeamBreakSensor(secondSensorChannel, null);

    public StraightenatorSubsystem(StraightenatorIO io) {
        m_io = io.withFirstSensor(m_firstSensor).withSecondSensor(m_secondSensor);

        OurRobotState.addScoreMechanismStateChangeCallback(this::scoreMechanismStateChangeCallback);
    }

    @Override
    public void periodic() {
        m_io.periodic();
        if (m_inputs.firstSensorTripped && OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE)
            m_io.off();

        if (m_inputs.secondSensorTripped)
            m_io.off();

        OurRobotState.setIsCoralHolderFirstSensorTripped(m_inputs.firstSensorTripped);
        OurRobotState.setIsCoralHolderSecondSensorTripped(m_inputs.secondSensorTripped);
        OurRobotState.setIsCoralInHolder(m_inputs.firstSensorTripped);

        m_io.updateInputs(m_inputs);

        Logger.processInputs("StraightenatorSubsystem", m_inputs);
    }

    public Command on() {
        return runOnce(m_io::on);
    }
    public Command off() {
        return runOnce(m_io::off);
    }

    private void scoreMechanismStateChangeCallback() {
        switch (OurRobotState.getScoreMechanismState()) {
            case HOME:
                off().schedule();
                break;
            case CORAL_INTAKE:
                on().schedule();
                break;
            default:
                break;
        }
    }
}
