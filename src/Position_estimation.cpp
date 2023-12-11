#include  "Position_estimation.h"
#include "Encoders.h"

Encoder RomiEncoders;
unsigned long time_prev = millis();
unsigned long time_now = 0;
const float deltaT = 0.05;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {
        //assignment
        //double mathfiller = (pow(x,2) + pow(y,2))
        //float c = sqrt(double(pow(x,2) + pow(y,2)));
        
        float Vr = target_speed_right;
        float Vl = target_speed_left;
        float R = (l/2)*((Vr+Vl)/(Vr-Vl));
        float w = (Vr-Vl)/l;
        float V = (Vl + Vr) / 2;
        if(target_speed_left == target_speed_right) 
        {
            //pose_data myPose;
            x = x + V*cos(theta)*deltaT;
            y = y + V*sin(theta)*deltaT;
            theta = theta;
        }
        else {
            x = x - R*sin(theta) + R*sin(theta + w*deltaT);
            y  = y + R*cos(theta) - R*cos(theta + w*deltaT);
            theta = theta + w*deltaT;
        }
        time_prev = time_now;
        return; 
        }
    }


