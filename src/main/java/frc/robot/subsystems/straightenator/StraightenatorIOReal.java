// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import static frc.robot.subsystems.straightenator.StraightenatorConstants.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.OurRobotState;
import frc.robot.OurRobotState.ScoreMechanismState;

public class StraightenatorIOReal implements StraightenatorIO {
    private final TalonFX m_leftMotor = new TalonFX(leftMotorId);
    private final TalonFX m_rightMotor = new TalonFX(rightMotorId);

    private final DigitalInput m_firstSensor = new DigitalInput(firstSensorId);
    private final DigitalInput m_secondSensor = new DigitalInput(secondSensorId);

    private final NeutralOut m_neturalRequest = new NeutralOut();

    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(intakeOutput);

    @Override
    public void periodic() {
        boolean firstSensorTripped = !m_firstSensor.get();
        boolean secondSensorTripped = !m_secondSensor.get();

        if (firstSensorTripped && OurRobotState.getScoreMechanismState() == ScoreMechanismState.CORAL_INTAKE)
            off();

        if (secondSensorTripped)
            off();

        OurRobotState.setIsCoralHolderFirstSensorTripped(firstSensorTripped);
        OurRobotState.setIsCoralHolderSecondSensorTripped(secondSensorTripped);
    }

    @Override
    public void updateInputs(StraightenatorIOInputs inputs) {
        
    }

    @Override
    public void on() {
        m_leftMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
        m_rightMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
    }
    @Override
    public void off() {
        m_leftMotor.setControl(m_neturalRequest);
        m_rightMotor.setControl(m_neturalRequest);
    }
}
