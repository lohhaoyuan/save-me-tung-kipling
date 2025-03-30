#ifndef MOVEMENT_H
#define MOBEMENT_H

#include "pid.h"
#include "shared.h"
#include "config.h"
#include "vector.h"


// Direction parameters
namespace Direction {
    // Move in a certain direction
    struct Constant {
        double value = 0;
    };
    // Move towards a certain point on the field
    struct MoveToPoint {
        Vector robot;
        Point destination;
    };
    // Move down a line
    struct LineTrack {
        Line line;
        double targetLineDepth = 0.5;
        bool trackLeftwards = true; // track left of an angle bisector from the
                                    // line facing into the field
    };
    } // namespace Direction
    
    // Velocity parameters
    namespace Velocity {
    // Move at a certain velocity
    struct Constant {
        double value = 0;
    };
    // Decelerate to a stop at a certain point
    struct StopAtPoint {
        double error;
        double minSpeed;
        double maxSpeed;
    };
    // Impose maximum limits on the components of velocity (relative to the field)
    struct Limit {
        double maxX;
        double maxY;
    };
    } // namespace Velocity
    
    // Heading parameters
    namespace Heading {
    // Move at a certain heading
    struct Constant {
        double value = 0;
    };
    
    // Approach a target heading as the robot approaches a target point
    struct MoveToPoint {
        Vector robot;
        Point destination;
        double targetHeading = 0;
    };
    } // namespace Heading
    
    class Movement {
      public:
        Movement();
    
        void init();
    
        // Update controllers at the start of the loop
        void updateHeading(const double robotHeading);
    
        // Set movement parameters
        void setDirection(const Direction::Constant params);
        void setDirection(const Direction::MoveToPoint params);
        void setDirection(const Direction::LineTrack params);
    
        void setVelocity(const Velocity::Constant params);
        void setVelocity(const Velocity::StopAtPoint params);
        void setVelocity(const Velocity::Limit params);
    
        void setHeading(const Heading::Constant params);
        void setHeading(const Heading::MoveToPoint params);
    
        void setSolenoidActive();
    
        // Actuate outputs at the end of the loop
        void drive();
    
        // Controllers (these are public so that we can run debugPrint() on them)
        PIDController headingController = PIDController(
            0,                                  // Target angle
            -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED,  // Output limits
            HEADING_KP, HEADING_KI, HEADING_KD, // Gains
            HEADING_MIN_DT, HEADING_MAXI, HEADING_MAX_SETPOINT_CHANGE);
        PIDController lineTrackController =
            PIDController(0,     // Target lateral distance offset
                          -1, 1, // Output limits
                          LINE_TRACK_KP, LINE_TRACK_KI, LINE_TRACK_KD, // Gains
                          LINE_TRACK_MIN_DT, LINE_TRACK_MAXI);
        PIDController stopAtPointController = PIDController(
            0,                                 // Target distance offset
            -DRIVE_MAX_SPEED, DRIVE_MAX_SPEED, // Output limits
            STOP_AT_POINT_KP, STOP_AT_POINT_KI, STOP_AT_POINT_KD, // Gains
            STOP_AT_POINT_MIN_DT, STOP_AT_POINT_MAXI);
    
      private:
        // Controller input
        double actualHeading = 0;
    
        // Parameters
        double direction = 0;
        double velocity = 0;
        double heading = 0;
    
        // Used by Direction::LineTrack
        bool isLineTracking = false;
        // Used by Direction::MoveToPoint and Heading::MoveToPoint
        bool isMovingToPoint = false;
        Point moveToPointDestination;
        double moveToPointInitialHeading;
        Vector moveToPointStart;
        // Used by Velocity::StopAtPoint
        bool isStoppingAtPoint = false;
        // Used by solenoid
        uint32_t solenoidLastActivated = 0;
    
      public:
        // Utility functions
        static double applySigmoid(const double startSpeed, const double endSpeed,
                                   const double progress);
    };
    

#endif