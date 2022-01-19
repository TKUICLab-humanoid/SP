#include "walkinggait/walkinggait.h"

void ChangeContinuousValue(const tku_msgs::Interface& msg)
{
    walkdata.Walking_Cmd = etChangeValue;// start = 0x01, stop = 0x02, change value = 0x04
    walkdata.X = msg.x;
    walkdata.Y = msg.y;
    walkdata.Z = msg.z;
    if(abs(msg.theta) > 20)
    {
        walkdata.THETA = pre_theta;
    }
    else
    {
        walkdata.THETA = msg.theta;
        pre_theta = msg.theta;
    }
    walkdata.Sensor_Mode = msg.sensor_mode;
    std::string temp;
    if(walkdata.Sensor_Mode == 0)
    {
        temp = "None";
    }
    else if(walkdata.Sensor_Mode == 1)
    {
        temp = "Pitch";
    }
    else if(walkdata.Sensor_Mode == 2)
    {
        temp = "Roll";
    }
    else if(walkdata.Sensor_Mode == 3)
    {
        temp = "RollPitch";
    }
    ROS_INFO("Change Value [%-6d %-6d %-4d; %s]", walkdata.X, walkdata.Y, walkdata.THETA, temp.c_str());
    if((pre_x<0 && msg.x>=0)||(pre_x>=0 && msg.x<0))
    {
        printf("pre_x:%d\tx:%d\n", pre_x, msg.x);
        char line[SIZE];
        fstream fin;
        char path[200];
        strcpy(path, tool->parameterPath.c_str());

        if(parameterinfo->walking_mode == 1)//reload
        {
            if(msg.x >= 0)
            {
                strcat(path, "/Continuous_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Back.ini");
            }
            fin.open(path,ios::in);
            fin.getline(line,sizeof(line),'\n');

            parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
            parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
            parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
            parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
            parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
            parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
            parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
            parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        }
        else if( parameterinfo->walking_mode == 7 )
        {
            if(msg.x >= 0)
            {
                strcat(path, "/Continuous_Second_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Second_Back.ini");
            }
            fin.open(path,ios::in);
            fin.getline(line,sizeof(line),'\n');

            parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
            parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
            parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
            parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
            parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
            parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
            parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
            parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        }
        else if( parameterinfo->walking_mode == 8 )
        {
            if(msg.x >= 0)
            {
                strcat(path, "/Continuous_Third_Parameter.ini");
            }
            else
            {
                strcat(path, "/Continuous_Third_Back.ini");
            }
            fin.open(path,ios::in);
            fin.getline(line,sizeof(line),'\n');

            parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
            parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
            parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
            parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
            parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
            parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
            parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
            parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        }
        paradata.Walking_Mode = parameterinfo->walking_mode;
        paradata.X_Swing_Range = parameterinfo->parameters.X_Swing_Range;
        paradata.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range;
        paradata.Z_Swing_Range = parameterinfo->parameters.Z_Swing_Range;
        paradata.Period_T = parameterinfo->parameters.Period_T;
        paradata.Period_T2 = parameterinfo->parameters.Period_T2;
        paradata.Sample_Time = parameterinfo->parameters.Sample_Time;
        paradata.OSC_LockRange = parameterinfo->parameters.OSC_LockRange;
        paradata.BASE_Default_Z = parameterinfo->parameters.BASE_Default_Z;
        paradata.Y_Swing_Shift = parameterinfo->parameters.Y_Swing_Shift;
        paradata.X_Swing_COM = parameterinfo->parameters.X_Swing_COM;
        paradata.BASE_LIFT_Z = parameterinfo->parameters.BASE_LIFT_Z;
        paradata_Pub.publish(paradata);
        printf("YSWING:%f\n", parameterinfo->parameters.Y_Swing_Range);
        tool->Delay(25);
    }
    pre_x = msg.x;

    walkdata_Pub.publish(walkdata);
    tool->Delay(50);
}
void ContinousMode(const std_msgs::Int16& msg)
{
    ContMode = msg.data;
}
string DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}
void save_parameter(const tku_msgs::parameter& msg)
{
    string savedText;
    fstream fp;
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
    switch (msg.mode)
    {
    case 4:
    case 0:
        strcat(path, "/Single_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    case 1:
        if(continuousback_flag)
            strcat(path, "/Continuous_Back.ini");
        else
            strcat(path, "/Continuous_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 7:
        if(continuousback_flag)
            strcat(path, "/Continuous_Second_Back.ini");
        else
            strcat(path, "/Continuous_Second_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 8:
        if(continuousback_flag)
            strcat(path, "/Continuous_Third_Back.ini");
        else
            strcat(path, "/Continuous_Third_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z);
        fp<<savedText;
        break;
    case 2:
        strcat(path, "/LCstep_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "X_Swing_COM = " + DtoS(msg.X_Swing_COM) + "\n";
        fp<<savedText;
        savedText = "BASE_LIFT_Z = " + DtoS(msg.BASE_LIFT_Z) + "\n";
        fp<<savedText;
        savedText = "Stand_Balance = " + DtoS(msg.Stand_Balance);
        fp<<savedText;
        break;
    case 3:
        strcat(path, "/LCdown_Parameter.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "X_Swing_COM = " + DtoS(msg.X_Swing_COM) + "\n";
        fp<<savedText;
        savedText = "BASE_LIFT_Z = " + DtoS(msg.BASE_LIFT_Z);
        fp<<savedText;
        break;
    case 5:
        strcat(path, "/Single_wood.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    case 6:
        strcat(path, "/Single_third.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "X_Swing_Range = " + DtoS(msg.X_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Z_Swing_Range = " + DtoS(msg.Z_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Period_T2 = " + DtoS(msg.Period_T2) + "\n";
        fp<<savedText;
        savedText = "Sample_Time = " + DtoS(msg.Sample_Time) + "\n";
        fp<<savedText;
        savedText = "OSC_LockRange = " + DtoS(msg.OSC_LockRange) + "\n";
        fp<<savedText;
        savedText = "BASE_Default_Z = " + DtoS(msg.BASE_Default_Z) + "\n";
        fp<<savedText;
        savedText = "Y_Swing_Shift = " + DtoS(msg.Y_Swing_Shift);
        fp<<savedText;
        break;
    case 9:
        strcat(path, "/Right_Kick_Ball.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_X = " + DtoS(msg.B_SplineParam.Kick_Point_X) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_Y = " + DtoS(msg.B_SplineParam.Kick_Point_Y) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_Z = " + DtoS(msg.B_SplineParam.Kick_Point_Z) + "\n";
        fp<<savedText;
        savedText = "Back_Point_X = " + DtoS(msg.B_SplineParam.Back_Point_X) + "\n";
        fp<<savedText;
        savedText = "Back_Point_Z = " + DtoS(msg.B_SplineParam.Back_Point_Z) + "\n";
        fp<<savedText;
        savedText = "Support_Foot_Hip_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Hip_Upper_Pitch) + "\n";
        fp<<savedText;
        savedText = "Kick_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch) + "\n";
        fp<<savedText;
        savedText = "Support_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Ankle_Upper_Pitch) + "\n";
        fp<<savedText;
        break;
    case 10:
        strcat(path, "/Left_Kick_Ball.ini");
        fp.open(path, ios::out);
        savedText = "[General]\n";
        fp<<savedText;
        savedText = "Y_Swing_Range = " + DtoS(msg.Y_Swing_Range) + "\n";
        fp<<savedText;
        savedText = "Period_T = " + DtoS(msg.Period_T) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_X = " + DtoS(msg.B_SplineParam.Kick_Point_X) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_Y = " + DtoS(msg.B_SplineParam.Kick_Point_Y) + "\n";
        fp<<savedText;
        savedText = "Kick_Point_Z = " + DtoS(msg.B_SplineParam.Kick_Point_Z) + "\n";
        fp<<savedText;
        savedText = "Back_Point_X = " + DtoS(msg.B_SplineParam.Back_Point_X) + "\n";
        fp<<savedText;
        savedText = "Back_Point_Z = " + DtoS(msg.B_SplineParam.Back_Point_Z) + "\n";
        fp<<savedText;
        savedText = "Support_Foot_Hip_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Hip_Upper_Pitch) + "\n";
        fp<<savedText;
        savedText = "Kick_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch) + "\n";
        fp<<savedText;
        savedText = "Support_Foot_Ankle_Upper_Pitch = " + DtoS(msg.B_SplineParam.Support_Foot_Ankle_Upper_Pitch) + "\n";
        fp<<savedText;
        break;
    }
    fp.close();
}
void Getparameter(const tku_msgs::Interface& msg)
{
    char line[SIZE];
    fstream fin;
    parameterinfo->X = (double)msg.x/1000;
    parameterinfo->Y = (double)msg.y/1000;
    parameterinfo->Z = (double)msg.z/1000;
    parameterinfo->THETA = (double)msg.theta/180*PI;
    parameterinfo->walking_mode = msg.walking_mode;
    parameterinfo->sensor_mode = msg.sensor_mode;

    pre_x = parameterinfo->X;

    char path[200];
    strcpy(path, tool->parameterPath.c_str());

    if( parameterinfo->walking_mode == 1 )//reload
    {
        ROS_INFO("Generate Continuous");
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else if( parameterinfo->walking_mode == 2 )
    {
        ROS_INFO("Generate LCstep");
        strcat(path, "/LCstep_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);

        parameterinfo->parameters.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        parameterinfo->parameters.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
        parameterinfo->parameters.Stand_Balance = tool->readvalue(fin,"Stand_Balance",1);
    }
    else if( parameterinfo->walking_mode == 3 )
    {
        ROS_INFO("Generate LCdown");
        strcat(path, "/LCdown_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);

        parameterinfo->parameters.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        parameterinfo->parameters.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
    }
    else if(parameterinfo->walking_mode == 5)
    {
        ROS_INFO("Generate Single_wood");
        strcat(path, "/Single_wood.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;
    }
    else if(parameterinfo->walking_mode == 6)
    {
        ROS_INFO("Generate Single_third");
        strcat(path, "/Single_third.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;
    }
    else if( parameterinfo->walking_mode == 7 )
    {
        ROS_INFO("Generate Continuous_Second");
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Second_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Second_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else if( parameterinfo->walking_mode == 8 )
    {
        ROS_INFO("Generate Continuous_Third");
        if(parameterinfo->X >= 0)
        {
            strcat(path, "/Continuous_Third_Parameter.ini");
        }
        else
        {
            strcat(path, "/Continuous_Third_Back.ini");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
    }
    else if( parameterinfo->walking_mode == 9 )
    {
        ROS_INFO("Generate Right_Kick_Ball");
        strcat(path, "/Right_Kick_Ball.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        parameterinfo->parameters.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        parameterinfo->parameters.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        parameterinfo->parameters.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        parameterinfo->parameters.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
    }
    else if( parameterinfo->walking_mode == 10 )
    {
        ROS_INFO("Generate Left_Kick_Ball");
        strcat(path, "/Left_Kick_Ball.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        parameterinfo->parameters.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        parameterinfo->parameters.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        parameterinfo->parameters.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        parameterinfo->parameters.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
    }
    else
    {
        ROS_INFO("Generate Single");
        strcat(path, "/Single_Parameter.ini");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        parameterinfo->parameters.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        parameterinfo->parameters.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        parameterinfo->parameters.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        parameterinfo->parameters.Period_T = tool->readvalue(fin,"Period_T",0);
        parameterinfo->parameters.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        parameterinfo->parameters.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        parameterinfo->parameters.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        parameterinfo->parameters.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        parameterinfo->parameters.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        if((parameterinfo->Y >= 1) || (parameterinfo->Y <= -1))
            parameterinfo->parameters.Y_Swing_Range -= parameterinfo->parameters.Y_Swing_Shift;

    }

    walkdata.Walking_Cmd = etStop;

    paradata.Walking_Mode = parameterinfo->walking_mode;
    paradata.X_Swing_Range = parameterinfo->parameters.X_Swing_Range;
    paradata.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range;
    paradata.Z_Swing_Range = parameterinfo->parameters.Z_Swing_Range;
    paradata.Period_T = parameterinfo->parameters.Period_T;
    paradata.Period_T2 = parameterinfo->parameters.Period_T2;
    paradata.Sample_Time = parameterinfo->parameters.Sample_Time;
    paradata.OSC_LockRange = parameterinfo->parameters.OSC_LockRange;
    paradata.BASE_Default_Z = parameterinfo->parameters.BASE_Default_Z;
    paradata.Y_Swing_Shift = parameterinfo->parameters.Y_Swing_Shift;
    paradata.X_Swing_COM = parameterinfo->parameters.X_Swing_COM;
    paradata.BASE_LIFT_Z = parameterinfo->parameters.BASE_LIFT_Z;
    paradata.Stand_Balance = parameterinfo->parameters.Stand_Balance;
    paradata.B_SplineParam.Kick_Point_X = parameterinfo->parameters.Kick_Point_X;
    paradata.B_SplineParam.Kick_Point_Y = parameterinfo->parameters.Kick_Point_Y;
    paradata.B_SplineParam.Kick_Point_Z = parameterinfo->parameters.Kick_Point_Z;
    paradata.B_SplineParam.Back_Point_X = parameterinfo->parameters.Back_Point_X;
    paradata.B_SplineParam.Back_Point_Z = parameterinfo->parameters.Back_Point_Z;
    paradata.B_SplineParam.Support_Foot_Hip_Upper_Pitch = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    paradata.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    paradata.B_SplineParam.Support_Foot_Ankle_Upper_Pitch = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    paradata_Pub.publish(paradata);
    tool->Delay(25);

    walkdata.X = msg.x;
    walkdata.Y = msg.y;
    walkdata.Z = msg.z;
    walkdata.THETA = msg.theta;
    walkdata.Sensor_Mode = msg.sensor_mode;
    walkdata_Pub.publish(walkdata);
    tool->Delay(100);
}

bool LoadWalkingGaitParameterFunction(tku_msgs::WalkingGaitParameter::Request &req, tku_msgs::WalkingGaitParameter::Response &res)
{
    char line[SIZE];
    fstream fin;
    char path[200];
    strcpy(path, tool->parameterPath.c_str());

    switch(req.mode)
    {
    case 4:
    case 0:
        strcat(path, "/Single_Parameter.ini");
        ROS_INFO("Load Single_Parameter");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    case 1:
        if(continuousback_flag)
        {
            strcat(path, "/Continuous_Back.ini");
            ROS_INFO("Load Continuous_Back");
        }
        else
        {
            strcat(path, "/Continuous_Parameter.ini");
            ROS_INFO("Load Continuous_Parameter");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 7:
        if(continuousback_flag)
        {
            strcat(path, "/Continuous_Second_Back.ini");
            ROS_INFO("Load Continuous_Second_Back");
        }
        else
        {
            strcat(path, "/Continuous_Second_Parameter.ini");
            ROS_INFO("Load Continuous_Second_Parameter");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 8:
        if(continuousback_flag)
        {
            strcat(path, "/Continuous_Third_Back.ini");
            ROS_INFO("Load Continuous_Third_Back");
        }
        else
        {
            strcat(path, "/Continuous_Third_Parameter.ini");
            ROS_INFO("Load Continuous_Third_Parameter");
        }
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        break;
    case 2:
        strcat(path, "/LCstep_Parameter.ini");
        ROS_INFO("Load LCstep_Parameter");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
	    res.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
        break;
    case 3:
        strcat(path, "/LCdown_Parameter.ini");
        ROS_INFO("Load LCdown_Parameter");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.X_Swing_COM = tool->readvalue(fin,"X_Swing_COM",1);
        res.BASE_LIFT_Z = tool->readvalue(fin,"BASE_LIFT_Z",1);
        break;
    case 5:
        strcat(path, "/Single_wood.ini");
        ROS_INFO("Load Single_wood");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    case 6:
        strcat(path, "/Single_third.ini");
        ROS_INFO("Load Single_third");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.X_Swing_Range = tool->readvalue(fin,"X_Swing_Range",1);
        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Z_Swing_Range = tool->readvalue(fin,"Z_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.Period_T2 = tool->readvalue(fin,"Period_T2",0);
        res.Sample_Time = tool->readvalue(fin,"Sample_Time",0);
        res.OSC_LockRange = tool->readvalue(fin,"OSC_LockRange",1);
        res.BASE_Default_Z = tool->readvalue(fin,"BASE_Default_Z",1);
        res.Y_Swing_Shift = tool->readvalue(fin,"Y_Swing_Shift",1);
        break;
    case 9:
        strcat(path, "/Right_Kick_Ball.ini");
        ROS_INFO("Load Right_Kick_Ball");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.B_SplineParam.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        res.B_SplineParam.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        res.B_SplineParam.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        res.B_SplineParam.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        res.B_SplineParam.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        res.B_SplineParam.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        res.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        res.B_SplineParam.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
        break;
    case 10:
        strcat(path, "/Left_Kick_Ball.ini");
        ROS_INFO("Load Left_Kick_Ball");
        fin.open(path,ios::in);
        fin.getline(line,sizeof(line),'\n');

        res.Y_Swing_Range = tool->readvalue(fin,"Y_Swing_Range",1);
        res.Period_T = tool->readvalue(fin,"Period_T",0);
        res.B_SplineParam.Kick_Point_X = tool->readvalue(fin,"Kick_Point_X",1);
        res.B_SplineParam.Kick_Point_Y = tool->readvalue(fin,"Kick_Point_Y",1);
        res.B_SplineParam.Kick_Point_Z = tool->readvalue(fin,"Kick_Point_Z",1);
        res.B_SplineParam.Back_Point_X = tool->readvalue(fin,"Back_Point_X",1);
        res.B_SplineParam.Back_Point_Z = tool->readvalue(fin,"Back_Point_Z",1);
        res.B_SplineParam.Support_Foot_Hip_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Hip_Upper_Pitch",1);
        res.B_SplineParam.Kick_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Kick_Foot_Ankle_Upper_Pitch",1);
        res.B_SplineParam.Support_Foot_Ankle_Upper_Pitch = tool->readvalue(fin,"Support_Foot_Ankle_Upper_Pitch",1);
        break;
    }

    return true;
}

void ContinuousbackFunction(const std_msgs::Bool& msg)
{
    continuousback_flag = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WalkingGait");
	ros::NodeHandle nh;

    ContinousMode_subscriber = nh.subscribe("/ContinousMode_Topic", 2, ContinousMode);
    ChangeContinuousValue_subscriber = nh.subscribe("/ChangeContinuousValue_Topic", 2, ChangeContinuousValue);
    chatter_subscriber = nh.subscribe("/SendBodyAuto_Topic", 2, Getparameter);
    save_parameter_subscriber = nh.subscribe("/web/parameter_Topic", 100, save_parameter);
    continuousback_subscriber = nh.subscribe("/walkinggait/Continuousback", 100, ContinuousbackFunction);
    LoadWalkinggaitParameter_service = nh.advertiseService("/web/LoadWalkingGaitParameter", LoadWalkingGaitParameterFunction);

    paradata_Pub = nh.advertise<tku_msgs::Parameter_message>("/package/parameterdata", 1);
    walkdata_Pub = nh.advertise<tku_msgs::Walking_message>("/package/walkingdata", 2);

    ros::spin();

    return 0;
}
