#ifndef SSLCONFIG_H
#define SSLCONFIG_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>

namespace SSLConfig
{
    class Field
    {
    private:
        int robotsCount = 22;
        int robotsBlueCount = 11;
        int robotsYellowCount = 11;
        int fieldType = 0;
        double fieldLineWidth = 0.01;
        double fieldLength = 12.00;
        double fieldWidth = 9.00;
        double fieldRad = 0.50;
        double fieldFreeKick = 0.70;
        double fieldPenaltyWidth = 3.60;
        double fieldPenaltyDepth = 1.80;
        double fieldPenaltyPoint = 8.00;
        double fieldMargin = 0.3;
        double fieldRefereeMargin = 0.0;
        double wallThickness = 0.05;
        double goalThickness = 0.02;
        double goalDepth = 0.18;
        double goalWidth = 1.8;
        double goalHeight = 0.16;
        void setFieldLineWidth(double value) { this->fieldLineWidth = value; }
        void setFieldLength(double value) { this->fieldLength = value; }
        void setFieldWidth(double value) { this->fieldWidth = value; }
        void setFieldRad(double value) { this->fieldRad = value; }
        void setFieldFreeKick(double value) { this->fieldFreeKick = value; }
        void setFieldPenaltyWidth(double value) { this->fieldPenaltyWidth = value; }
        void setFieldPenaltyDepth(double value) { this->fieldPenaltyDepth = value; }
        void setFieldPenaltyPoint(double value) { this->fieldPenaltyPoint = value; }
        void setFieldMargin(double value) { this->fieldMargin = value; }
        void setFieldRefereeMargin(double value) { this->fieldRefereeMargin = value; }
        void setWallThickness(double value) { this->wallThickness = value; }
        void setGoalThickness(double value) { this->goalThickness = value; }
        void setGoalDepth(double value) { this->goalDepth = value; }
        void setGoalWidth(double value) { this->goalWidth = value; }
        void setGoalHeight(double value) { this->goalHeight = value; }

    public:
        int getRobotsCount() { return this->robotsCount; }
        int getRobotsBlueCount() { return this->robotsBlueCount; }
        int getRobotsYellowCount() { return this->robotsYellowCount; }
        double getFieldLineWidth() { return this->fieldLineWidth; }
        double getFieldLength() { return this->fieldLength; }
        double getFieldWidth() { return this->fieldWidth; }
        double getFieldRad() { return this->fieldRad; }
        double getFieldFreeKick() { return this->fieldFreeKick; }
        double getFieldPenaltyWidth() { return this->fieldPenaltyWidth; }
        double getFieldPenaltyDepth() { return this->fieldPenaltyDepth; }
        double getFieldPenaltyPoint() { return this->fieldPenaltyPoint; }
        double getFieldMargin() { return this->fieldMargin; }
        double getFieldRefereeMargin() { return this->fieldRefereeMargin; }
        double getWallThickness() { return this->wallThickness; }
        double getGoalThickness() { return this->goalThickness; }
        double getGoalDepth() { return this->goalDepth; }
        double getGoalWidth() { return this->goalWidth; }
        double getGoalHeight() { return this->goalHeight; }
        int getFieldType() { return this->fieldType; }
        void setRobotsCount(int value) { this->robotsCount = value; }
        void setRobotsBlueCount(int value) { this->robotsBlueCount = value; }
        void setRobotsYellowCount(int value) { this->robotsYellowCount = value; }
        void setFieldType(int value)
        {
            this->fieldType = value;
            switch (this->fieldType)
            {
            case 0: // Division A
                setRobotsCount(22);
                setRobotsBlueCount(11);
                setRobotsYellowCount(11);
                setFieldLength(12.00);
                setFieldWidth(9.00);
                setFieldPenaltyWidth(3.60);
                setFieldPenaltyDepth(1.80);
                setFieldPenaltyPoint(8.00);
                setGoalWidth(1.80);
                break;
            case 1: // Division B
                setRobotsCount(12);
                setRobotsBlueCount(6);
                setRobotsYellowCount(6);
                setFieldLength(9.00);
                setFieldWidth(6.00);
                setFieldPenaltyWidth(2.00);
                setFieldPenaltyDepth(1.00);
                setFieldPenaltyPoint(6.00);
                setGoalWidth(1.00);
                break;
            case 2: // Hardware challenge
                setRobotsCount(12);
                setRobotsBlueCount(6);
                setRobotsYellowCount(6);
                setFieldLength(6.00);
                setFieldWidth(4.00);
                setFieldPenaltyWidth(2.00);
                setFieldPenaltyDepth(0.80);
                setFieldPenaltyPoint(6.00);
                setGoalWidth(0.70);
                break;
            default:
                break;
            }
        }
    };

    class World
    {
    private:
        int ballSlip = 1;
        bool resetTurnOver = true;
        double ballRadius = 0.0215;
        double gravity = 9.81;
        double ballMass = 0.043;
        double ballFriction = 0.05;
        double ballBounce = 0.5;
        double ballBounceVel = 0.1;
        double ballLinearDamp = 0.004;
        double ballAngularDamp = 0.004;

    public:
        int getBallSlip() { return this->ballSlip; }
        bool getResetTurnOver() { return this->resetTurnOver; }
        double getBallRadius() { return this->ballRadius; }
        double getGravity() { return this->gravity; }
        double getBallMass() { return this->ballMass; }
        double getBallFriction() { return this->ballFriction; }
        double getBallBounce() { return this->ballBounce; }
        double getBallBounceVel() { return this->ballBounceVel; }
        double getBallLinearDamp() { return this->ballLinearDamp; }
        double getBallAngularDamp() { return this->ballAngularDamp; }
    };

    class Robot
    {
    private:
        int wheel0Angle = 60;
        int wheel1Angle = 135;
        int wheel2Angle = 225;
        int wheel3Angle = 300;
        double radius = 0.090;
        double height = 0.146;
        double bottomHeight = 0.004;
        double wheelRadius = 0.02475;
        double wheelThickness = 0.005;
        double bodyMass = 2.200;
        double wheelMass = 0.050;
        double wheelTangentFriction = 0.800;
        double wheelPerpendicularFriction = 0.050;
        double wheelMotorMaxTorque = 0.070; // Maxon EC45 flat 50w with 18:60 gear ratio, reduced to 3m/s2 accel
        double wheelMotorMaxRPM = 1557.0; // Maxon EC45 flat 50w with 18:60 gear ratio
        
        double distanceCenterKicker = 0.081;
        double kickerZ = 0.005;
        double kickerThickness = 0.005;
        double kickerWidth = 0.080;
        double kickerHeight = 0.040;
        double kickerMass = 0.020;
        double kickerDampFactor = 0.200;
        double kickerFriction = 0.800;
        double kickerAngle = acos(distanceCenterKicker/radius);
        double rollerTorqueFactor = 0.060;
        double rollerPerpendicularTorqueFactor = 0.005;


    public:
        int getWheel0Angle() { return this->wheel0Angle; }
        int getWheel1Angle() { return this->wheel1Angle; }
        int getWheel2Angle() { return this->wheel2Angle; }
        int getWheel3Angle() { return this->wheel3Angle; }
        double getDistanceCenterKicker() { return this->distanceCenterKicker; }
        double getKickerZ() { return this->kickerZ; }
        double getKickerThickness() { return this->kickerThickness; }
        double getKickerWidth() { return this->kickerWidth; }
        double getKickerHeight() { return this->kickerHeight; }
        double getKickerMass() { return this->kickerMass; }
        double getKickerDampFactor() { return this->kickerDampFactor; }
        double getKickerFriction() { return this->kickerFriction; }
        double getKickerAngle() { return this->kickerAngle; }
        double getDribblerTorqueFactor() { return this->rollerTorqueFactor; }
        double getDribblerPerpendicularTorqueFactor() { return this->rollerPerpendicularTorqueFactor; }
        double getRadius() { return this->radius; }
        double getHeight() { return this->height; }
        double getBottomHeight() { return this->bottomHeight; }
        double getWheelRadius() { return this->wheelRadius; }
        double getWheelThickness() { return this->wheelThickness; }
        double getBodyMass() { return this->bodyMass; }
        double getWheelMass() { return this->wheelMass; }
        double getWheelTangentFriction() { return this->wheelTangentFriction; }
        double getWheelPerpendicularFriction() { return this->wheelPerpendicularFriction; }
        double getWheelMotorMaxTorque() { return this->wheelMotorMaxTorque; }
        double getWheelMotorMaxRPM() { return this->wheelMotorMaxRPM; }

    };

} // namespace Config

#endif