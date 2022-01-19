#ifndef TOOL_H
#define TOOL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sys/time.h>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
#include <iostream>
#include <string>

using namespace std;

class Tool
{
    public:
        Tool();
        ~Tool();

        string getPackagePath(string package_name);
        void initParameterPath();
        float readvalue(fstream &fin, string title,int mode);
        void Delay(int timedelay);

    public:
        string parameterPath;
        char standPath[200];

	private:
		string packagePath;
};

class ToolInstance : public Tool
{
	public:
		ToolInstance() : Tool(){}
		~ToolInstance(){}
		static ToolInstance* getInstance();
		static void deleteInstance();

	private:
		static ToolInstance *m_pInstance;
};

class TimeClass
{
    public:
        TimeClass(double check_time_ms);
        TimeClass();
        ~TimeClass();
        void updateTime();
        void initialize();
        void setTimerPass(double check_time_ms, bool init_flag = true);
        double getTimeMs();
		double getPeriodTimeMs();
        bool checkTimePass();

    private:
        double start;
        double end;
        double time_ms;
        double check_time_ms;
};

#endif // TOOL_H
