#include "tdataunit/tdataunit.hpp"

TdataUnit::TdataUnit()
{
    CameraParameterValue = new CameraParameter;
    CameraParameterValue->auto_white_balance = 0;
    CameraParameterValue->auto_exposure = 0;
    CameraParameterValue->auto_Backlight_Compensation = 0;
    CameraParameterValue->brightness = 0;
    CameraParameterValue->contrast = 0;
    CameraParameterValue->saturation = 0;
    CameraParameterValue->white_balance = 0;
    CameraParameterValue->ParameterName = "[Camera Set Parameter]";   
    tool = ToolInstance::getInstance();
}

TdataUnit::~TdataUnit()
{
    delete CameraParameterValue;
}

void TdataUnit::SaveCameraSetFile()
{
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
    strcat(path, "/CameraSet.ini");
    try
    {
        ofstream OutFile(path);
        OutFile << CameraParameterValue->ParameterName;
        OutFile << "\n";
        OutFile << "brightness = ";
        OutFile << CameraParameterValue->brightness;
        OutFile << "\n";
        OutFile << "contrast = ";
        OutFile << CameraParameterValue->contrast;
        OutFile << "\n";
        OutFile << "saturation = ";
        OutFile << CameraParameterValue->saturation;
        OutFile << "\n";
        OutFile << "white_balance = ";
        OutFile << CameraParameterValue->white_balance;
        OutFile << "\n";
        OutFile << "auto_white_balance = ";
        OutFile << CameraParameterValue->auto_white_balance;
        OutFile << "\n";
        OutFile << "auto_exposure = ";
        OutFile << CameraParameterValue->auto_exposure;
        OutFile << "\n";
        OutFile << "auto_Backlight_Compensation = ";
        OutFile << CameraParameterValue->auto_Backlight_Compensation;
        OutFile << "\n";

        OutFile.close();
    }
    catch( exception e )
    {
    }
}

void TdataUnit::LoadCameraSetFile()
{
    fstream fin;
    char line[100]; 
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
    strcat(path, "/CameraSet.ini");
    fin.open(path, ios::in);
    try
    {
        fin.getline(line,sizeof(line),'\n');
        CameraParameterValue->brightness = tool->readvalue(fin, "brightness", 0);
        CameraParameterValue->contrast = tool->readvalue(fin, "contrast", 0);
        CameraParameterValue->saturation = tool->readvalue(fin, "saturation", 0);
        CameraParameterValue->white_balance = tool->readvalue(fin, "white_balance", 0);
        CameraParameterValue->auto_white_balance = tool->readvalue(fin, "auto_white_balance", 0);
        CameraParameterValue->auto_exposure = tool->readvalue(fin, "auto_exposure", 0);
        CameraParameterValue->auto_Backlight_Compensation = tool->readvalue(fin, "auto_Backlight_Compensation", 0);
        fin.close();
    }
    catch(exception e)
    {
    }
}


