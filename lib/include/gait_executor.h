#include <math.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <regex>
#include <sstream>
#include <Gait.h>




class GaitExecutor {
    public:
        // Constructor
        GaitExecutor();

        // Destructor
        ~GaitExecutor() = default;

        /**
         * Pose_Update (Position Update)
         * 
         * Increments the precent_step and calls Command_Body to move to its next position along its movement.
         * 
         * @param event It's just a timer.
        */
        void Pose_Update();

        /**
         * Takes a point centered on the body's origin and recenters it on the origin of a given leg.
         * @param point The current point centered on the body
         * @param leg The specific leg to be recented about
         * @returns The new point recentered about the specified leg
        */
        Point recenter_point(Point point, int leg);

        /**
         * 
        */

        // Operation Methods

        /**
         * Updates the internaly stored leg locations.
         * 
        */
        void set_leg_positions(Gait gait);

        /**
         * Command_SR (Command Superior Ritgh Leg)
         * 
         * Updates the positon of the superior right leg.
        */
        void Command_SR();

        /**
         * Command_SL (Command Superior Left Leg)
         * 
         * Updates the positon of the superior left leg.
        */
        void Command_SL();

        /**
         * Command_IR (Command Inferior Right Leg)
         * 
         * Updates the positon of the inferior right leg.
        */
        void Command_IR();

       /**
         * Command_IL (Command inferior Left Leg)
         * 
         * Updates the positon of the inferior left leg.
        */
        void Command_IL();

        /**
         * Computes the desired elevation of the lifted foot
         */
        Gait gait_raise_foot(Gait gait);

        /**
         * Command_Body
         * 
         *  Linearily interpolate and normalize the current gait position.
        */
        void Command_Body();

        /**
         * Init
         * 
         * Sets the robot to its default idle standing position.
        */
        void Init();
        Gait normalize_gait(Gait gait);
        Gait gait_lerp(Gait g1, Gait g2, double percent);

        // Debugging
        void debug(std::vector<double> values, std::string message);
        void debug(std::string message);
        void print_gait(Gait gait);
};
/**
 * double_lerp (Linear Interpolation double type)
 * @param x1 the current position value
 * @param x2 the desired position value
 * @param percent the percentage completion of the linear interpolation in decimal form
*/
double double_lerp(double x1, double x2, double percent);

/**
 * point_lerp (Linear Interpolation Point type)
 * @param x1 the current position value
 * @param x2 the desired position value
 * @param percent the percentage completion of the linear interpolation in decimal form
*/
Point point_lerp(Point p1, Point p2, double percent);