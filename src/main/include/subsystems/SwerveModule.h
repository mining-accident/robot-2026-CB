#pragma once

// hardware includes
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

// wpilib includes
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/length.h>

// project includes

using namespace ctre::phoenix6;

class swerveModule {
public:
    // Constructor
    swerveModule (int azimuthEncoderID, int azimuthMotorID, int driveMotorID);

    // Module state setters
    void setDriveOpenLoop(double driveVoltage);
    void setSteerOpenLoop(double steerVoltage);
    void setModuleState(frc::SwerveModuleState moduleState);
    void setModuleMode();

    // Module state getters
    frc::SwerveModuleState getModuleState();
    frc::SwerveModuleState getModuleError();
    frc::SwerveModulePosition getModulePosition();

private:
    // Module hardware
    hardware::CANcoder m_azimuthEncoder;
    hardware::TalonFX m_azimuthMotor;
    hardware::TalonFX m_driveMotor;

    // Module physical constants
    frc::Translation2d m_modulePosition;
    units::meter_t m_wheelDiameter;
    double m_driveRatio;
    double m_steerRatio;

    // Module variables
    frc::SwerveModuleState m_desiredState; // Module setpoint (m/s, degrees)
    frc::SwerveModuleState m_currentState; // Module current state (m/s, degrees)
    frc::SwerveModulePosition m_currentPosition; // Module current position (x, y)
};