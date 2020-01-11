package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.command.Drive;


/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

    private static final double kGearRatio = 5.39;
    private static final double kWheelRadiusMeters = 0.127;

    public DifferentialDriveOdometry driveOdometry;
    public DifferentialDriveKinematics differentialDriveKinematics;
    public RamseteController ramseteController;

    public final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    public final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    public static final CANSparkMax rightWheelsMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax rightWheelsSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static final CANSparkMax leftWheelsMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax leftWheelsSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    public static final Solenoid shifter = new Solenoid(0);

    public Drivetrain() {
        differentialDriveKinematics = new DifferentialDriveKinematics(0.7);
        driveOdometry = new DifferentialDriveOdometry(getAngle()); //FIXME

        ahrs.reset();
        ahrs.zeroYaw();

        ramseteController = new RamseteController(2.0, 0.7);

        motorSetUpTeleop();

    }

    @Override
    public void periodic() {
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftWheelsMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60,
                rightWheelsMaster.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60
        );
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-ahrs.getYaw());
    }

    public void reset() {
        ahrs.zeroYaw();
    }

    public void setSpeeds(double leftPower, double rightPower) {
        rightWheelsMaster.set(rightPower);
        leftWheelsMaster.set(leftPower);
    }

    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftWheelsMaster.getPIDController().setReference(leftVoltage, ControlType.kVoltage);
        rightWheelsMaster.getPIDController().setReference(rightVoltage, ControlType.kVoltage);
    }

    public void motorSetUpTeleop() {
        leftWheelsMaster.restoreFactoryDefaults();
        leftWheelsSlave.restoreFactoryDefaults();
        rightWheelsMaster.restoreFactoryDefaults();
        rightWheelsSlave.restoreFactoryDefaults();

        leftWheelsMaster.setInverted(true);

        leftWheelsSlave.setIdleMode(IdleMode.kCoast);
        leftWheelsMaster.setIdleMode(IdleMode.kBrake);
        rightWheelsSlave.setIdleMode(IdleMode.kCoast);
        rightWheelsMaster.setIdleMode(IdleMode.kBrake);

        leftWheelsSlave.follow(leftWheelsMaster);
        rightWheelsSlave.follow(rightWheelsMaster);

        leftWheelsMaster.setOpenLoopRampRate(0);
        leftWheelsSlave.setOpenLoopRampRate(0);
        rightWheelsMaster.setOpenLoopRampRate(0);
        rightWheelsSlave.setOpenLoopRampRate(0);

        leftWheelsMaster.setSmartCurrentLimit(38);
        leftWheelsSlave.setSmartCurrentLimit(38);
        rightWheelsMaster.setSmartCurrentLimit(38);
        rightWheelsSlave.setSmartCurrentLimit(38);
    }

    public void shiftUp() {
        if (!shifter.get()) {
            System.out.println("Shifted Up");
            shifter.set(true);
        }
    }

    public void shiftDown() {
        if (shifter.get()) {
            System.out.println("Shifted Down");
            shifter.set(false);
        }
    }

    @Override
    public String toString() {
        return "Drivetrain{" +
                "} " + super.toString();
    }
}