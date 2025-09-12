// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.straightenator;

import static frc.robot.subsystems.straightenator.StraightenatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.BeamBreakSensor;

public class StraightenatorIOReal implements StraightenatorIO {
    private final TalonFX m_leftMotor = new TalonFX(leftMotorId, Constants.CANBUS);
    private final TalonFX m_rightMotor = new TalonFX(rightMotorId, Constants.CANBUS);

    private final StatusSignal<AngularVelocity> m_leftMotorVelocitySignal = m_leftMotor.getVelocity();
    private final StatusSignal<Current> m_leftMotorCurrentSignal = m_leftMotor.getSupplyCurrent();
    private double m_leftMotorCurrent = 0d;

    private final StatusSignal<AngularVelocity> m_rightMotorVelocitySignal = m_rightMotor.getVelocity();
    private final StatusSignal<Current> m_rightMotorCurrentSignal = m_rightMotor.getSupplyCurrent();
    private double m_rightMotorCurrent = 0d;

    private BeamBreakSensor m_firstSensor;
    private BeamBreakSensor m_secondSensor;
    private boolean m_firstSensorTripped;
    private boolean m_secondSensorTripped;

    private boolean m_isOn = false;
    private Command m_antiJamCommand = null;

    private final NeutralOut m_neutralRequest = new NeutralOut();

    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(intakeOutput);

    public StraightenatorIOReal() {
        var leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = neutralMode;
        leftConfig.MotorOutput.Inverted = leftInverted;

        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = currentLimit;

        var rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = neutralMode;
        rightConfig.MotorOutput.Inverted = rightInverted;

        rightConfig.CurrentLimits = leftConfig.CurrentLimits;


        tryUntilOk(5, () -> m_leftMotor.getConfigurator().apply(leftConfig));
        tryUntilOk(5, () -> m_rightMotor.getConfigurator().apply(rightConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_leftMotorVelocitySignal, m_leftMotorCurrentSignal, m_rightMotorVelocitySignal, m_rightMotorCurrentSignal);
    }

    @Override
    public StraightenatorIOReal withFirstSensor(BeamBreakSensor sensor) {
        m_firstSensor = sensor;
        return this;
    }
    @Override
    public StraightenatorIOReal withSecondSensor(BeamBreakSensor sensor) {
        m_secondSensor = sensor;
        return this;
    }

    @Override
    public void updateInputs(StraightenatorIOInputs inputs) {
        m_firstSensorTripped = !m_firstSensor.get();
        m_secondSensorTripped = !m_secondSensor.get();

        inputs.firstSensorTripped = m_firstSensorTripped;
        inputs.secondSensorTripped = m_secondSensorTripped;

        BaseStatusSignal.refreshAll(m_leftMotorVelocitySignal, m_leftMotorCurrentSignal, m_rightMotorVelocitySignal, m_rightMotorCurrentSignal);

        inputs.leftMotorVelocityRPS = m_leftMotorVelocitySignal.getValueAsDouble();
        m_leftMotorCurrent = m_leftMotorCurrentSignal.getValueAsDouble();
        inputs.leftMotorCurrentAmps = m_leftMotorCurrent;

        inputs.rightMotorVelocityRPS = m_rightMotorVelocitySignal.getValueAsDouble();
        m_rightMotorCurrent = m_rightMotorCurrentSignal.getValueAsDouble();
        inputs.rightMotorCurrentAmps = m_rightMotorCurrent;
    }

    private class AntiJamCommand extends Command {
        private TalonFX m_motor;
        private final Timer m_timer = new Timer();

        public AntiJamCommand(TalonFX talonFX) {
            m_motor = talonFX;
        }

        @Override
        public void initialize() {
            m_timer.restart();
            m_motor.setControl(m_dutyCycleRequest.withOutput(antiJamReverseOutput));

            // m_leftMotor.setControl(m_dutyCycleRequest.withOutput(antiJamReverseOutput));
            // m_rightMotor.setControl(m_dutyCycleRequest.withOutput(antiJamReverseOutput));
        }

        @Override
        public boolean isFinished() {
            return m_timer.hasElapsed(antiJamDurationSeconds);
        }

        @Override
        public void end(boolean interuppted) {
            m_motor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));

            // m_leftMotor.setControl(m_neutralRequest);
            // m_rightMotor.setControl(m_neutralRequest);
        }
    }

    @Override
    public void periodic() {
        if (m_isOn && m_antiJamCommand == null) {
            if (m_leftMotorCurrent >= antiJamCurrentThreshold) {
                m_antiJamCommand = new AntiJamCommand(m_leftMotor);
                m_antiJamCommand.schedule();
            } else if (m_rightMotorCurrent >= antiJamCurrentThreshold) {
                m_antiJamCommand = new AntiJamCommand(m_rightMotor);
                m_antiJamCommand.schedule();
            }
        }
    }

    @Override
    public void on() {
        m_isOn = true;
        if (ENABLE) {
            m_leftMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
            m_rightMotor.setControl(m_dutyCycleRequest.withOutput(intakeOutput));
        }
    }
    @Override
    public void off() {
        m_isOn = false;

        if (m_antiJamCommand != null) {
            m_antiJamCommand.cancel();
            m_antiJamCommand = null;
        }

        if (ENABLE) {
            m_leftMotor.setControl(m_neutralRequest);
            m_rightMotor.setControl(m_neutralRequest);
        }
    }
}
