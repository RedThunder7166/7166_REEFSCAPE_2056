// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import frc.robot.util.BeamBreakSensor;

public class StraightenatorIOSim implements StraightenatorIO {
    private BeamBreakSensor m_firstSensor;
    private BeamBreakSensor m_secondSensor;

    @Override
    public StraightenatorIOSim withFirstSensor(BeamBreakSensor sensor) {
        m_firstSensor = sensor;

        return this;
    }
    @Override
    public StraightenatorIOSim withSecondSensor(BeamBreakSensor sensor) {
        m_secondSensor = sensor;

        return this;
    }

    @Override
    public void updateInputs(StraightenatorIOInputs inputs) {
        inputs.firstSensorTripped = !m_firstSensor.get();
        inputs.secondSensorTripped = !m_secondSensor.get();
    }
}
