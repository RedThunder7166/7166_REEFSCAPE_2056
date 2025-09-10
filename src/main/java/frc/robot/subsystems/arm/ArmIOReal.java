// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.util.BeamBreakSensor;

public class ArmIOReal implements ArmIO {
    private final TalonFX m_pivotMotor = new TalonFX(pivotMotorId, Constants.CANBUS);
    private final TalonFX m_gripperMotor = new TalonFX(gripperMotorId, Constants.CANBUS);
    private BeamBreakSensor m_gripperSensor;
    private boolean m_gripperSensorTripped = false;

    private final StatusSignal<Angle> m_pivotMotorPositionSignal = m_pivotMotor.getPosition();
    private final StatusSignal<Current> m_pivotMotorCurrentSignal = m_pivotMotor.getSupplyCurrent();

    private double m_pivotMotorTargetPosition = pivotPositionInitial;

    private final NeutralOut m_neutralRequest = new NeutralOut();

    private final DutyCycleOut m_gripperDutyCycleRequest = new DutyCycleOut(0d);

    private final MotionMagicVoltage m_pivotPositionRequest = new MotionMagicVoltage(m_pivotMotorTargetPosition);

    private boolean m_gripperCoralOn = false;

    public ArmIOReal() {
        var pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = pivotNeutralMode;
        pivotConfig.MotorOutput.Inverted = pivotInverted;

        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = pivotCurrentLimit;
        pivotConfig.Feedback.SensorToMechanismRatio = pivotMotorReduction;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kP = pidP;
        pivotConfig.MotionMagic.MotionMagicAcceleration = acceleration;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;

        var gripperConfig = new TalonFXConfiguration();
        gripperConfig.MotorOutput.NeutralMode = gripperNeutralMode;
        gripperConfig.MotorOutput.Inverted = gripperInverted;

        gripperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        gripperConfig.CurrentLimits.SupplyCurrentLimit = gripperCurrentLimit;

        tryUntilOk(5, () -> m_pivotMotor.getConfigurator().apply(pivotConfig));
        tryUntilOk(5, () -> m_gripperMotor.getConfigurator().apply(gripperConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(50d, m_pivotMotorPositionSignal, m_pivotMotorCurrentSignal);

        m_pivotMotor.setPosition(m_pivotMotorTargetPosition);
    }

    @Override
    public ArmIOReal withGripperSensor(BeamBreakSensor sensor) {
        m_gripperSensor = sensor;
        return this;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        m_gripperSensorTripped = !m_gripperSensor.get();
        inputs.gripperSensorTripped = m_gripperSensorTripped;

        BaseStatusSignal.refreshAll(m_pivotMotorPositionSignal, m_pivotMotorCurrentSignal);

        final double pivotMotorPositionRotations = m_pivotMotorPositionSignal.getValueAsDouble();
        inputs.pivotMotorPositionRotations = pivotMotorPositionRotations;
        inputs.pivotMotorPositionDegrees = mechanismPositionToPivotAngle(pivotMotorPositionRotations).in(Degrees);
        inputs.pivotTargetMotorPositionRotations = m_pivotMotorTargetPosition;
        inputs.pivotMotorCurrentAmps = m_pivotMotorCurrentSignal.getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (m_gripperCoralOn && m_gripperSensorTripped)
            gripperOff();
    }

    @Override
    public void pivotGoPosition(double position) {
        // FIXME: ELEVATOR CLEARANCE
        m_pivotMotorTargetPosition = position;
        // FIXME: TEMPORARY
        // m_pivotMotor.setControl(m_pivotPositionRequest.withPosition(m_pivotMotorTargetPosition));
    }

    @Override
    public boolean pivotIsAtPosition(double position) {
        return Math.abs(position - m_pivotMotorPositionSignal.getValueAsDouble()) <= pivotIsAtPositionThreshold;
    }

    @Override
    public boolean pivotIsAtOrPastPosition(double position) {
        return (m_pivotMotorPositionSignal.getValueAsDouble() - position) >= pivotIsAtPositionThreshold;
    }

    @Override
    public void gripperCoralOn() {
        m_gripperCoralOn = true;
        // FIXME: TEMPORARY
        // m_gripperMotor.setControl(m_gripperDutyCycleRequest.withOutput(gripperCoralOutput));
    }
    @Override
    public void gripperAlgaeOn() {
        // FIXME: TEMPORARY
        // m_gripperMotor.setControl(m_gripperDutyCycleRequest.withOutput(gripperAlgaeOutput));
    }

    @Override
    public void gripperReverse() {
        m_gripperCoralOn = false;
        // FIXME: TEMPORARY
        // m_gripperMotor.setControl(m_gripperDutyCycleRequest.withOutput(gripperReverseOutput));
    }

    @Override
    public void gripperOff() {
        m_gripperCoralOn = false;
        // FIXME: TEMPORARY
        // m_gripperMotor.setControl(m_neutralRequest);
    }
}
