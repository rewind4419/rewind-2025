#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "SwerveConstants.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>


#include <stdio.h>

using namespace pathplanner;

/*
CAN IDs (on the canivore)

fr and bl are swapped
4 and 5 should be 2 and 3

FL Steer - 0
FL Drive - 1
FR Steer - 2
FR Drive - 3
BL Steer - 4
BL Drive - 5
BR Steer - 6
BR Drive - 7

9 and 10 need to swap

FL Encoder - 8
FR Encoder - 9
BL Encoder - 10
BR Encoder - 11

IMU Pigeon 2 - 12

*/

class CommandSwerveDrivetrain : public frc2::SubsystemBase, public TunerSwerveDrivetrain {
    static constexpr units::second_t kSimLoopPeriod = 5_ms;
    std::unique_ptr<frc::Notifier> m_simNotifier;
    units::second_t m_lastSimTime;

    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    static constexpr frc::Rotation2d kBlueAlliancePerspectiveRotation{0_deg};
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    static constexpr frc::Rotation2d kRedAlliancePerspectiveRotation{180_deg};
    /* Keep track if we've ever applied the operator perspective before or not */
    bool m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    swerve::requests::SysIdSwerveTranslation m_translationCharacterization;
    swerve::requests::SysIdSwerveSteerGains m_steerCharacterization;
    swerve::requests::SysIdSwerveRotation m_rotationCharacterization;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    frc2::sysid::SysIdRoutine m_sysIdRoutineTranslation{
        frc2::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            4_V,          // Reduce dynamic step voltage to 4 V to prevent brownout
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdTranslation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output) { SetControl(m_translationCharacterization.WithVolts(output)); },
            {},
            this
        }
    };

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    frc2::sysid::SysIdRoutine m_sysIdRoutineSteer{
        frc2::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            7_V,          // Use dynamic voltage of 7 V
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdSteer_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output) { SetControl(m_steerCharacterization.WithVolts(output)); },
            {},
            this
        }
    };

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of swerve::requests::SysIdSwerveRotation for info on importing the log to SysId.
     */
    frc2::sysid::SysIdRoutine m_sysIdRoutineRotation{
        frc2::sysid::Config{
            /* This is in radians per second², but SysId only supports "volts per second" */
            units::constants::detail::PI_VAL / 6 * (1_V / 1_s),
            /* This is in radians per second, but SysId only supports "volts" */
            units::constants::detail::PI_VAL * 1_V,
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdRotation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output)
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                SetControl(m_rotationCharacterization.WithRotationalRate(output * (1_rad_per_s / 1_V)));
                /* also log the requested output for SysId */
                SignalLogger::WriteValue("Rotational_Rate", output * (1_rad_per_s / 1_V));
            },
            {},
            this
        }
    };

    /* The SysId routine to test */
    frc2::sysid::SysIdRoutine *m_sysIdRoutineToApply = &m_sysIdRoutineTranslation;

public:
    swerve::requests::ApplyRobotSpeeds m_applyRobotSpeeds = swerve::requests::ApplyRobotSpeeds {};

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * \param modules             Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, ModuleConstants const &... modules) :
        TunerSwerveDrivetrain{driveTrainConstants, modules...}
    {
        printf("Swerve it?\n");

        if (utils::IsSimulation()) {
            StartSimThread();
        }

        printf("Configuring AutoBuilder...\n");

        frc::DCMotor br = frc::DCMotor::KrakenX60();
        

        RobotConfig config = RobotConfig {50_kg, 6.8_kg_sq_m, ModuleConfig {
            0.0508_m,
            1_mps,
            1.2,
            br.WithReduction(5.361),
            40_A,
            1
        }, 0.533_m};

        // Configure the AutoBuilder last
        AutoBuilder::configure(
            [this](){
                // add a cached robot pose
                printf("Giving the pose: %f, %f\n", this->GetState().Pose.X().value(), this->GetState().Pose.Y().value());
                return this->GetState().Pose;
            }, // Robot pose supplier
            [this](frc::Pose2d pose){
                printf("Resetting pose to X: %f, Y: %f, R: %f\n", pose.X().value(), pose.Y().value(), pose.Rotation().Radians());
                this->ResetPose(pose);
            }, // Method to reset odometry (will be called if your auto has a starting pose)
            [this](){ printf("Providing speeds: %f, %f\n", this->GetState().Speeds.vx.value(), this->GetState().Speeds.vy.value());  return this->GetState().Speeds; /*return getRobotRelativeSpeeds();*/ }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            [this](auto speeds, auto feedforwards){
                printf("Running at %f, %f\n", speeds.vx.value(), speeds.vy.value());
                frc::ChassisSpeeds sbpeeds{3.0_mps, -2.0_mps,
                  units::radians_per_second_t(std::numbers::pi)};

                this->SetControl(
                    this->m_applyRobotSpeeds.WithSpeeds(speeds)
                );
                // this->RunOnce([this](){
            
                //   frc::ChassisSpeeds sbpeeds{3.0_mps, -2.0_mps,
                //   units::radians_per_second_t(std::numbers::pi)};

                //   this->SetControl(
                //     this->m_applyRobotSpeeds.WithSpeeds(sbpeeds)
                //   );
                //   printf("Running the command\n");
                // });
                printf("Test driver\n");
            }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
                PIDConstants(10.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            []() {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                return false;
                // auto alliance = DriverStation::GetAlliance();
                // if (alliance) {
                //     return alliance.value() == DriverStation::Alliance::kRed;
                // }
                // return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        ModuleConstants const &... modules
    ) :
        TunerSwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency, modules...}
    {
        if (utils::IsSimulation()) {
            StartSimThread();
        }
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param odometryStandardDeviation  The standard deviation for odometry calculation
     * \param visionStandardDeviation    The standard deviation for vision calculation
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(
        swerve::SwerveDrivetrainConstants const &driveTrainConstants,
        units::hertz_t odometryUpdateFrequency,
        std::array<double, 3> const &odometryStandardDeviation,
        std::array<double, 3> const &visionStandardDeviation,
        ModuleConstants const &... modules
    ) :
        TunerSwerveDrivetrain{
            driveTrainConstants, odometryUpdateFrequency,
            odometryStandardDeviation, visionStandardDeviation, modules...
        }
    {
        if (utils::IsSimulation()) {
            StartSimThread();
        }
    }

    /*
     * \brief Returns a command that applies the specified control request to this swerve drivetrain.
     
     * This captures the returned swerve request by reference, so it must live
     * for at least as long as the drivetrain. This can be done by storing the
     * request as a member variable of your drivetrain subsystem or robot.
     
     * \param request Function returning the request to apply
     * \returns Command to run
     */
    template <typename RequestSupplier>
        requires std::is_lvalue_reference_v<std::invoke_result_t<RequestSupplier>> &&
            requires(RequestSupplier req, TunerSwerveDrivetrain &drive) { drive.SetControl(req()); }
    frc2::CommandPtr ApplyRequest(RequestSupplier request)
    {
        return Run([this, request=std::move(request)] {
            return SetControl(request());
        });
    }

    /**
     * \brief Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * \param request Function returning the request to apply
     * \returns Command to run
     */
    template <typename RequestSupplier>
        requires std::is_rvalue_reference_v<std::invoke_result_t<RequestSupplier>> &&
            requires(RequestSupplier req, TunerSwerveDrivetrain &drive) { drive.SetControl(req()); }
    frc2::CommandPtr ApplyRequest(RequestSupplier request)
    {
        return Run([this, request=std::move(request)] {
            return SetControl(request());
        });
    }

    void Periodic() override;

    /**
     * \brief Runs the SysId Quasistatic test in the given direction for the routine
     * specified by m_sysIdRoutineToApply.
     *
     * \param direction Direction of the SysId Quasistatic test
     * \returns Command to run
     */
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
    {
        return m_sysIdRoutineToApply->Quasistatic(direction);
    }

    /**
     * \brief Runs the SysId Dynamic test in the given direction for the routine
     * specified by m_sysIdRoutineToApply.
     *
     * \param direction Direction of the SysId Dynamic test
     * \returns Command to run
     */
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
    {
        return m_sysIdRoutineToApply->Dynamic(direction);
    }

private:
    void StartSimThread();
};
