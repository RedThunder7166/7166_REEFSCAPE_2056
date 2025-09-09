// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.OurRobotState.ScoreMechanismState;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.ground_intake.GroundIntakeIO;
import frc.robot.subsystems.ground_intake.GroundIntakeIOReal;
import frc.robot.subsystems.ground_intake.GroundIntakeIOSim;
import frc.robot.subsystems.ground_intake.GroundIntakeSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorIO;
import frc.robot.subsystems.straightenator.StraightenatorSubsystem;
import frc.robot.subsystems.straightenator.StraightenatorIOReal;
import frc.robot.subsystems.straightenator.StraightenatorIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive m_drive;
    private final GroundIntakeSubsystem m_groundIntakeSubsystem;
    private final StraightenatorSubsystem m_straightenatorSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ArmSubsystem m_armSubsystem;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                m_groundIntakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOReal());
                m_straightenatorSubsystem = new StraightenatorSubsystem(new StraightenatorIOReal());
                m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
                m_armSubsystem = new ArmSubsystem(new ArmIOReal());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                m_groundIntakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIOSim());
                m_straightenatorSubsystem = new StraightenatorSubsystem(new StraightenatorIOSim());
                m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
                m_armSubsystem = new ArmSubsystem(new ArmIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                m_groundIntakeSubsystem = new GroundIntakeSubsystem(new GroundIntakeIO() {});
                m_straightenatorSubsystem = new StraightenatorSubsystem(new StraightenatorIO() {});
                m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
                m_armSubsystem = new ArmSubsystem(new ArmIO() { });
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                    m_drive,
                    () -> -Controls.controller.getLeftY(),
                    () -> -Controls.controller.getLeftX(),
                    () -> -Controls.controller.getRightX()));

        /*
         * // Lock to 0° when A button is held
         * controller
         * .a()
         * .whileTrue(
         * DriveCommands.joystickDriveAtAngle(
         * drive,
         * () -> -controller.getLeftY(),
         * () -> -controller.getLeftX(),
         * () -> new Rotation2d()));
         *
         * // Switch to X pattern when X button is pressed
         * controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
         *
         * // Reset gyro to 0° when B button is pressed
         * controller
         * .b()
         * .onTrue(
         * Commands.runOnce(
         * () -> drive.setPose(
         * new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
         * drive)
         * .ignoringDisable(true));
         *
         */

        Controls.controller.start().onTrue(
            Commands.runOnce(
                () -> m_drive.setPose(
                    new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
            m_drive)
            .ignoringDisable(true));

        Controls.l1Coral.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.CORAL_SCORE_L1));
        Controls.l2Coral.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.CORAL_SCORE_L2));
        Controls.l3Coral.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.CORAL_SCORE_L3));
        Controls.l4Coral.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.CORAL_SCORE_L4));

        Controls.score.onTrue(OurRobotState.scoreCommand);
        Controls.l2Algae.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_PICKUP_L2));
        Controls.l3Algae.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_PICKUP_L3));
        Controls.algaePickupFloor.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_PICKUP_FLOOR));

        Controls.processor.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_SCORE_PROCESSOR));
        Controls.net.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_SCORE_NET));

        Controls.home.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.HOME));

        Controls.algaeHome.onTrue(
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.ALGAE_HOME));

        Controls.deployIntake.onTrue(
            // OurRobotState.deployIntake());
            OurRobotState.setScoreMechanismStateCommand(ScoreMechanismState.CORAL_INTAKE));
        Controls.retractIntake.onTrue(
            m_groundIntakeSubsystem.retract());

        Controls.grabCoral.onTrue(grabCoral());

        // TODO: we need this to be in a subsystem, obviously
        // Controls.deployClimb.onTrue(
        //     Commands.runOnce(
        //             () -> {
        //                 OurRobotState.deployClimb();
        //             }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private Command grabCoral() {
        return m_straightenatorSubsystem.on()
            .until(OurRobotState::getIsCoralHolderSecondSensorTripped)
            // straightenator stopped due to sensor automatically
            .andThen(m_armSubsystem.gripperCoralOn())
            .andThen(m_elevatorSubsystem.goPosition(ElevatorConstants.positionGrab))
            .until(OurRobotState::getIsCoralInGripper)
            .andThen(m_elevatorSubsystem.goPosition(ElevatorConstants.positionHome));
            // gripper stopped due to sensor automatically
    }
}
