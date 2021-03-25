#ifndef VSSCONFIG_H
#define VSSCONFIG_H

#include <vector>
#include <string>
#include <iostream>

namespace VSSConfig
{
    class Field
    {
    private:
        int RobotsCount = 3;
        int RobotsBlueCount = 3;
        int RobotsYellowCount = 3;
        int FieldType = 0;
        double FieldLineWidth = 0.003;
        double FieldLength = 1.50;
        double FieldWidth = 1.300;
        double FieldRad = 0.200;
        double FieldFreeKick = 0.200;
        double FieldPenaltyWidth = 0.70;
        double FieldPenaltyDepth = 0.15;
        double FieldPenaltyPoint = 0.35;
        double FieldMargin = 0.3;
        double FieldRefereeMargin = 0.4;
        double WallThickness = 0.025;
        double GoalThickness = 0.025;
        double GoalDepth = 0.10;
        double GoalWidth = 0.40;
        double GoalHeight = 0.05;
        void setFieldLineWidth(double value) { this->FieldLineWidth = value; }
        void setFieldLength(double value) { this->FieldLength = value; }
        void setFieldWidth(double value) { this->FieldWidth = value; }
        void setFieldRad(double value) { this->FieldRad = value; }
        void setFieldFreeKick(double value) { this->FieldFreeKick = value; }
        void setFieldPenaltyWidth(double value) { this->FieldPenaltyWidth = value; }
        void setFieldPenaltyDepth(double value) { this->FieldPenaltyDepth = value; }
        void setFieldPenaltyPoint(double value) { this->FieldPenaltyPoint = value; }
        void setFieldMargin(double value) { this->FieldMargin = value; }
        void setFieldRefereeMargin(double value) { this->FieldRefereeMargin = value; }
        void setWallThickness(double value) { this->WallThickness = value; }
        void setGoalThickness(double value) { this->GoalThickness = value; }
        void setGoalDepth(double value) { this->GoalDepth = value; }
        void setGoalWidth(double value) { this->GoalWidth = value; }
        void setGoalHeight(double value) { this->GoalHeight = value; }

    public:
        int getRobotsCount() { return this->RobotsCount; }
        int getRobotsBlueCount() { return this->RobotsBlueCount; }
        int getRobotsYellowCount() { return this->RobotsYellowCount; }
        double getFieldLineWidth() { return this->FieldLineWidth; }
        double getFieldLength() { return this->FieldLength; }
        double getFieldWidth() { return this->FieldWidth; }
        double getFieldRad() { return this->FieldRad; }
        double getFieldFreeKick() { return this->FieldFreeKick; }
        double getFieldPenaltyWidth() { return this->FieldPenaltyWidth; }
        double getFieldPenaltyDepth() { return this->FieldPenaltyDepth; }
        double getFieldPenaltyPoint() { return this->FieldPenaltyPoint; }
        double getFieldMargin() { return this->FieldMargin; }
        double getFieldRefereeMargin() { return this->FieldRefereeMargin; }
        double getWallThickness() { return this->WallThickness; }
        double getGoalThickness() { return this->GoalThickness; }
        double getGoalDepth() { return this->GoalDepth; }
        double getGoalWidth() { return this->GoalWidth; }
        double getGoalHeight() { return this->GoalHeight; }
        int getFieldType() { return this->FieldType; }
        void setRobotsCount(int value) { this->RobotsCount = value; }
        void setRobotsBlueCount(int value) { this->RobotsBlueCount = value; }
        void setRobotsYellowCount(int value) { this->RobotsYellowCount = value; }
        void setFieldType(int value)
        {
            this->FieldType = value;
            switch (this->FieldType)
            {
            case 0: // 3x3
                setFieldLength(1.50);
                setFieldWidth(1.300);
                setFieldRad(0.200);
                setFieldFreeKick(0.200);
                setFieldPenaltyWidth(0.70);
                setFieldPenaltyDepth(0.15);
                setFieldPenaltyPoint(0.35);
                break;
            case 1: // 5x5
                setFieldLength(2.2);
                setFieldWidth(1.8);
                setFieldRad(0.25);
                setFieldFreeKick(0.25);
                setFieldPenaltyWidth(0.8);
                setFieldPenaltyDepth(0.35);
                setFieldPenaltyPoint(0.375);
                break;
            default:
                break;
            }
        }
    };

    class World
    {
    private:
        double BallRadius = 0.0215;
        int DesiredFPS = 60;
        double Gravity = 9.81;
        bool ResetTurnOver = true;
        double BallMass = 0.043;
        double BallFriction = 0.01;
        int BallSlip = 1;
        double BallBounce = 0.5;
        double BallBounceVel = 0.1;
        double BallLinearDamp = 0.004;
        double BallAngularDamp = 0.004;

    public:
        double getBallRadius() { return this->BallRadius; }
        int getDesiredFPS() { return this->DesiredFPS; }
        double getGravity() { return this->Gravity; }
        bool getResetTurnOver() { return this->ResetTurnOver; }
        double getBallMass() { return this->BallMass; }
        double getBallFriction() { return this->BallFriction; }
        int getBallSlip() { return this->BallSlip; }
        double getBallBounce() { return this->BallBounce; }
        double getBallBounceVel() { return this->BallBounceVel; }
        double getBallLinearDamp() { return this->BallLinearDamp; }
        double getBallAngularDamp() { return this->BallAngularDamp; }
    };

    class Robot
    {
    private:
        double Radius = 0.0375;
        double Height = 0.056;
        double BottomHeight = 0.002;
        double WheelRadius = 0.026;
        double WheelThickness = 0.005;
        int Wheel0Angle = 90;
        int Wheel1Angle = 270;
        double BodyMass = 0.120;
        double WheelMass = 0.015;
        double WheelTangentFriction = 0.8;
        double WheelPerpendicularFriction = 1;
        double WheelMotorMaxTorque = 0.0725;
        double WheelMotorMaxRPM = 630.0;
        double casterWheelsRadius = 0.002;
        double casterWheelsMass = 0.001;

    public:
        double getRadius() { return this->Radius; }
        double getHeight() { return this->Height; }
        double getBottomHeight() { return this->BottomHeight; }
        double getWheelRadius() { return this->WheelRadius; }
        double getWheelThickness() { return this->WheelThickness; }
        int getWheel0Angle() { return this->Wheel0Angle; }
        int getWheel1Angle() { return this->Wheel1Angle; }
        double getBodyMass() { return this->BodyMass; }
        double getWheelMass() { return this->WheelMass; }
        double getWheelTangentFriction() { return this->WheelTangentFriction; }
        double getWheelPerpendicularFriction() { return this->WheelPerpendicularFriction; }
        double getWheelMotorMaxTorque() { return this->WheelMotorMaxTorque; }
        double getWheelMotorMaxRPM() { return this->WheelMotorMaxRPM; }
        double getCasterWheelsRadius() { return this->casterWheelsRadius; }
        double getCasterWheelsMass() { return this->casterWheelsMass; }

    };

} // namespace Config

#endif