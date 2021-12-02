#include "strategy/strategy_main.h"
int tao; 
int p=1;
int jing=0;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "kidsize");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy(nh);

	ros::Rate loop_rate(30);

    KidsizeStrategy.initparameterpath();

	while (nh.ok()) 
	{

		KidsizeStrategy.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
          
void KidsizeStrategy::strategymain()
{
	if(strategy_info->getStrategyStart())
	{	
        SprintInfo->IMU_now = strategy_info->getIMUValue().Yaw;   //獲得IMU的Yaw值
        printf("\nIMU_now= %1.5f\n",SprintInfo->IMU_now);
        printf("\nIMU_start= %1.5f\n",SprintInfo->IMU_start);
        printf("\nIMU_right= %1.5f\n",SprintInfo->IMU_right);
        printf("\nIMU_left= %1.5f\n",SprintInfo->IMU_left);
        printf("\n time_flag=%d\n",time_flag);
	    //printf("\n tao=%d\n",tao);
	    printf("\nsend_x=%d\n",SprintInfo->SpintInfomation->send_x);
        ROS_INFO(" now totalsize = %d", total_size);
        
		strategy_info->get_image_flag = true;
		ros_com->drawImageFunction(1, DrawMode::DrawLine, 0, 320, 120, 120, 152, 245, 255);
		ros_com->drawImageFunction(2, DrawMode::DrawLine, 160, 160, 0, 240, 152, 245, 255);
		ros_com->drawImageFunction(3, DrawMode::DrawObject, 80, 240, 60, 180, 125, 38, 205);
		//ros_com->drawImageFunction(4, DrawMode::DrawObject, strategy_info->color_mask_subject[0][0].XMin, strategy_info->color_mask_subject[0][0].XMax, strategy_info->color_mask_subject[0][0].YMin, strategy_info->color_mask_subject[0][0].YMax, 255, 165, 0);
/****************************************************************************************************/
		once_flag = true;
        if(change_mode != 2)
        {
		    //strategy_info->get_image_flag = true;
		    //strategy_info->get_label_model_flag = true;
		    classify_strategy();										    // process image
		    head_strategy();
        }
		if (first_in == true)											//only once
		{
            // ros_com->sendSensorSet(0,6,0,0);
            SprintInfo->IMU_start = SprintInfo->IMU_now;
            SprintInfo->IMU_right = SprintInfo->IMU_start - 3;                                   
            SprintInfo->IMU_left  = SprintInfo->IMU_start + 3;
			ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
			tool->Delay(10);
            first_in = false;
			time_flag = true;
			get_timeuse_flag = true;
			timeuse = 0;
		}
        if(change_mode == 2)                                            //if change_mode == 2 means aruco
        {	tao=0;
			aruco_head_strategy();
            aruco_move_strategy();
            ROS_INFO("send_theta = %1.5f", SprintInfo->SpintInfomation->send_theta);
     		ROS_INFO("aruco_distence = %1.5f", aruco_distence);
     		ROS_INFO("aruco_direction = %1.5f", aruco_direction);
        }
        else                                                            //if change_mode == 0 1 means watch ball
        {
		    SprintInfo->fram_count++;
		    if(SprintInfo->SpintInfomation->SprForWard) SprintInfo->fram_count++;
		    //if (SprintInfo->SpintInfomation->get_target || (change_mode == 1 && SprintInfo->SpintInfomation->SprForWard == 0))//if mode==1 and back it will be true
		    //{
            if (SprintInfo->SpintInfomation->get_target)
            {
		        if (SprintInfo->SpintInfomation->SprForWard) //是否前進中
		        {
		    		do_forward();
		    		if (total_size > back_total_size) //判斷是否後退走
		    		{
		    			ROS_INFO("~~~~~~turn Backward~~~~~~");
		    			SprintInfo->SpintInfomation->checkback_cnt++;
		    			SprintInfo->SpintInfomation->send_x = slowdown_initial_x; //步態X軸參數
		    		}
		    		else
		    		{
		    			SprintInfo->SpintInfomation->checkback_cnt = 0;
		    		}
		    		if (SprintInfo->SpintInfomation->checkback_cnt > CHECKBACK_CNT) //如回程條件成立
		    		{
                        tool->Delay(500);
		    			SprintInfo->SpintInfomation->SprForWard = false; //close forward
		    			SprintInfo->SpintInfomation->tmp_theta = backward_initial_theta;
		    			SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
		    			SprintInfo->SpintInfomation->send_x = backward_initial_x; //步態X軸參數
		    		}
		        }
		        else
		        {
                    do_backward_ybrat();
                    /*
		    		ROS_INFO("Do Backward!!!!!!");
		    		if(change_mode == 1) 
		    		{	
		    		    if(time_flag == true) 
		    	        {
                            if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
                            {
                                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_right_three;
                                 
                                printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                            }
	    				    else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
                            {
                                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_left_three;
                                
                                printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                            }
                            else
                            {
                              time_flag = false;
                              ros_com->sendSensorSet(0,-8,0,0,0,0,0);  //IMU修正
                            }
                        }
                     
	    			    else
                        {
                            gettimeofday(&tstart, NULL);   
                            do_backward_without_watch_ball(without_left,without_right);
                        } 
                    }
		    		else 
		    		{
		    			do_backward_ybrat();
		    		}
                    */
		        }
            }
            else
            {
                if (SprintInfo->SpintInfomation->SprForWard)
                {
                    do_forward();
                }
                else
                {
                    do_backward_ybrat();
                }
            }
		    //}
		    //else   //沒抓到目標
		    /*{
		    	ROS_INFO("lost target!\n");
		    	if (SprintInfo->SpintInfomation->SprForWard)              //forward flag
		    	{
		    		if(YBrat > losttarget_ybrate) losttmp_theta = forward_initial_theta + losttarget_theta;
		    		else losttmp_theta = forward_initial_theta - losttarget_theta;
		    		SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x);  //send_x不超出forward_x_max，前進小於極限值
 		    	}
		    	else
		    	{
		    		SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
		    	}
		    }
		    if (SprintInfo->fram_count >= FRAMCOUNT)
		    {
		    	if (SprintInfo->SpintInfomation->tmp_theta == 0)
		    	{
		    		ROS_INFO("tmp_theta = %d  tmp_theta = 0 so send_theta = 0",SprintInfo->SpintInfomation->tmp_theta);
		    		SprintInfo->SpintInfomation->send_theta = 0;
		    	}
		    	else
		    	{
		    		if (SprintInfo->SpintInfomation->SprForWard)
		    		{
                        if(SprintInfo->SpintInfomation->get_target == false && total_size > REDSIZE_losttarget)
                        {
		    			    ROS_INFO("Forward & lost_target~~~~~~~~~~~~~~~");
                            SprintInfo->SpintInfomation->send_theta = losttmp_theta;
                            if(YBrat > losttarget_ybrate) ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2500, 1250);
		    		        else ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1844, 1250);
		    	            //ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1400, 1000);
                        }
                        else
                        {
                            ROS_INFO("Forward & get_target");
		    				ROS_INFO("forward fram_count = %d must be 4", SprintInfo->fram_count);
		    			    SprintInfo->SpintInfomation->send_theta += (int)SprintInfo->SpintInfomation->tmp_theta /(SprintInfo->fram_count / 2);				    //(*2)because need to belance double fram_count

		    			    SprintInfo->SpintInfomation->send_theta = min(Forward_ThetaMax + forward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn left Limit
		    			    SprintInfo->SpintInfomation->send_theta = max(Forward_ThetaMin + forward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn right Limit

		    			    SprintInfo->SpintInfomation->tmp_theta = forward_initial_theta;	//initializ
                        }
		    		}
		    		else if (change_mode == 0)
		    		{
		    			ROS_INFO("~~~~~~~~~~~~~yb you send information start~~~~~~~~~~~~~");
		    			ROS_INFO("fram_count = %2d",SprintInfo->fram_count);
		    			ROS_INFO("send_theta = %2d",SprintInfo->SpintInfomation->send_theta);
		    			ROS_INFO("tmp_theta = %2d",SprintInfo->SpintInfomation->tmp_theta);
		    			SprintInfo->SpintInfomation->send_theta += (int)SprintInfo->SpintInfomation->tmp_theta / SprintInfo->fram_count;
		    			ROS_INFO("send_theta = %2d",SprintInfo->SpintInfomation->send_theta);
		    			ROS_INFO("~~~~~~~~~~~~~yb you send information end~~~~~~~~~~~~~");
		    			SprintInfo->SpintInfomation->send_theta = min(Backward_ThetaMax + backward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn left Limit
		    			SprintInfo->SpintInfomation->send_theta = max(Backward_ThetaMin + backward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn right Limit

		    			SprintInfo->SpintInfomation->tmp_theta = backward_initial_theta;
		    			tool->Delay(100);
		    		}
		    	}
		    	check_headerror = true;
		    	SprintInfo->fram_count = 0;
		    }*/
		    //ROS_INFO("send_theta = %d", SprintInfo->SpintInfomation->send_theta);
		    //ROS_INFO("speed = %4d", SprintInfo->SpintInfomation->send_x);
		    //ROS_INFO("YBrat = %1.5f", YBrat);
		    //ROS_INFO("center_x = %f\n",SprintInfo->center_x);
            if (SprintInfo->SpintInfomation->SprForWard)
		    	{
		            ros_com->sendContinuousValue(SprintInfo->SpintInfomation->send_x, SprintInfo->SpintInfomation->send_y, 0, SprintInfo->SpintInfomation->send_theta,SensorMode::RollPitch);//imu:SensorMode::RollPitch
                }
            else
                {
                    ros_com->sendContinuousValue(SprintInfo->SpintInfomation->send_x, SprintInfo->SpintInfomation->send_y, 0, SprintInfo->SpintInfomation->send_theta,SensorMode::RollPitch);//imu:SensorMode::RollPitch
                }
		    tool->Delay(100); //for stability

		    Old_Toltal_Size = total_size;
        } 
	}
	else
	{
		load_strategy_parameter_ini();
		initial_strategy();
		if(once_flag)
		{
			if (first_in == false)
			{
				ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep,SensorMode::None);
				tool->Delay(10);
				first_in = true;
                time_flag = true;
			}
			once_flag = false;
		}
	}
}
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
/****************************************************************************************************/
void KidsizeStrategy::determine_object(void)
{
	int yellow_cnt, blue_cnt;
	int tmp_y_size=0,tmp_b_size=0;
    for (yellow_cnt = 0; yellow_cnt < strategy_info->color_mask_subject_cnts[(int)LabelModel::Yellow]; yellow_cnt++) //黃色數量
    {
        //ROS_INFO("Yc = %d",strategy_info->color_mask_subject_cnts[(int)LabelModel::Yellow]);
        /*for (blue_cnt = 0; blue_cnt < strategy_info->color_mask_subject_cnts[(int)LabelModel::Blue]; blue_cnt++) //藍色數量
        {
				if (strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax < strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMax) //Blue右邊界一定在Yellow之右
				{
					if (strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin < strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin) //Yellow左邊界一在Blue之左
					{
						if (abs((strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax - strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin) - (strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax - strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin))< 20) //Blue Yellow 高要接近
						{
							if (abs(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax - strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax) < 30)
							{
								if (abs(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin - strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin) < 30)
								{
									if (abs(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax - strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin) < 1.5 * (strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax - strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin)) //濾除斜的目標to be solved
									{
										if((strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax)<0||abs(strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax)<8)//兩色塊X軸距離需<8或交疊
										{
											if ((strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size)>50&&(strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].size) > 50)//黃色是否>50  藍色是否>50
											{
												if((strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax-strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin)>0&&(strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin)>0)//Ymax不低於另一側之Ymin
												{
													if(((strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax-strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin)/(strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMax-strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin))>(0.4))//右側(藍)色塊之Y/X 大於0.4(濾除扁形長方體雜訊)
													{*/
												if(((strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin)/(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin))==(1.0))//左側(黃)色塊之Y/X 大於0.4(濾除扁形長方體雜訊)
												{
                                                    if(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size<10000)
                                                    {
                                                        if(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size>1000)
                                                        {
															if(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size>tmp_y_size)	//選擇符合條件的物體中最大之目標物
                                                            {
																SprintInfo->SpintInfomation->ArrayYellow[etXMax] = strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax;
																SprintInfo->SpintInfomation->ArrayYellow[etXMin] = strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin;
																SprintInfo->SpintInfomation->ArrayYellow[etYMax] = strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax;
																SprintInfo->SpintInfomation->ArrayYellow[etYMin] = strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin;
                                                                ROS_INFO("Y:%d",(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin));
                                                                ROS_INFO("X:%d",(strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax-strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin));
                                                                
                                                                /*
																SprintInfo->SpintInfomation->ArrayBlue[etXMax] = strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMax; //攝影機看到的=ImageUnit->ObjectList[(int)LabelModel::Blue]->ObjectInfoList[blue_cnt].XMax
																SprintInfo->SpintInfomation->ArrayBlue[etXMin] = strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin;
																SprintInfo->SpintInfomation->ArrayBlue[etYMax] = strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax;
																SprintInfo->SpintInfomation->ArrayBlue[etYMin] = strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin;
                                                                */
																SprintInfo->SpintInfomation->ArrayYellow[etSize] = strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size;
																//SprintInfo->SpintInfomation->ArrayBlue[etSize] = strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].size;//record y and b size to compare next
																tmp_y_size=strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].size;
																//tmp_b_size=strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].size;

																ros_com->drawImageFunction(1, DrawMode::DrawObject, strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMin, strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].XMax, strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMin, strategy_info->color_mask_subject[(int)LabelModel::Yellow][yellow_cnt].YMax, 255, 255, 0);
																//ros_com->drawImageFunction(2, DrawMode::DrawObject, strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMin, strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].XMax, strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMin, strategy_info->color_mask_subject[(int)LabelModel::Blue][blue_cnt].YMax, 255, 255, 0);

																SprintInfo->SpintInfomation->get_target = true; //抓到目標物
															}
                                                        }
													}
												}
												/*}

											}
										}
									}
								}
							}
						}
					}
				}
        }*/
    }
tmp_y_size=0;
tmp_b_size=0;
}
/****************************************************************************************************/
void KidsizeStrategy::classify_strategy(void)
{    //使用的影像處理
    double tmpx, tmpy;

    SprintInfo->SpintInfomation->get_target = false;
    determine_object();

    if (SprintInfo->SpintInfomation->get_target)
    {
        //SprintInfo->center_y = ((SprintInfo->SpintInfomation->ArrayBlue[etYMin] + SprintInfo->SpintInfomation->ArrayBlue[etYMax]) / 2 +  (SprintInfo->SpintInfomation->ArrayYellow[etYMin] + SprintInfo->SpintInfomation->ArrayYellow[etYMax]) / 2) / 2; //雙色球中心點的y
        //SprintInfo->center_x = ((SprintInfo->SpintInfomation->ArrayBlue[etXMin] + SprintInfo->SpintInfomation->ArrayBlue[etXMax]) / 2 +  (SprintInfo->SpintInfomation->ArrayYellow[etXMin] + SprintInfo->SpintInfomation->ArrayYellow[etXMax]) / 2) / 2; //雙色球中心點的x
        SprintInfo->center_y = (SprintInfo->SpintInfomation->ArrayYellow[etYMin] + SprintInfo->SpintInfomation->ArrayYellow[etYMax]) / 2 ;//+  (SprintInfo->SpintInfomation->ArrayBlue[etYMin] + SprintInfo->SpintInfomation->ArrayBlue[etYMax]) / 2) / 2; //雙色球中心點的y
        SprintInfo->center_x = (SprintInfo->SpintInfomation->ArrayYellow[etXMin] + SprintInfo->SpintInfomation->ArrayYellow[etXMax]) / 2 ;// +  (SprintInfo->SpintInfomation->ArrayBlue[etXMin] + SprintInfo->SpintInfomation->ArrayBlue[etXMax]) / 2) / 2; //雙色球中心點的x
        total_size = SprintInfo->SpintInfomation->ArrayYellow[etSize]; //+ SprintInfo->SpintInfomation->ArrayBlue[etSize];
        // YBrat = (double)SprintInfo->SpintInfomation->ArrayBlue[etSize]/(double)SprintInfo->SpintInfomation->ArrayYellow[etSize];
        //YBrat = (double) (SprintInfo->SpintInfomation->ArrayBlue[etXMax]-SprintInfo->SpintInfomation->ArrayBlue[etXMin]) / (SprintInfo->SpintInfomation->ArrayYellow[etXMax] - SprintInfo->SpintInfomation->ArrayYellow[etXMin]);
        ROS_INFO(" old totalsize = %d", Old_Toltal_Size);
        ROS_INFO(" now totalsize = %d", total_size);
    }
}
/****************************************************************************************************/
bool KidsizeStrategy::head_strategy(void)
{
    if (SprintInfo->SpintInfomation->get_target)
    {
        if (SprintInfo->SpintInfomation->SprForWard)
        {
            //ROS_INFO("center_y = %f",SprintInfo->center_y);
            int move_vertical = (SprintInfo->center_y - 100);
            int tmpangle_y = 40.0 * (double)move_vertical / ImageLength;
            tool->Delay(40);
            //ROS_INFO("move_vertical = %d",move_vertical);
            //ROS_INFO("tmpangle_y = %d",tmpangle_y);
            if (abs(tmpangle_y) > 3)
            {
                SprintInfo->head_motor_y -= 11.378 * tmpangle_y * 0.4;
                if (SprintInfo->head_motor_y > 2210)								//2047 + 40*4+3 old:551
                {
                    SprintInfo->head_motor_y = 2210;
                }
                else if (SprintInfo->head_motor_y < 1100)							//2047 - 400*4+3 old:111
                {
                    SprintInfo->head_motor_y = 1100;
                }
            }
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 1000);			//將馬達y值改為上述head_motor_y值 old_speed:211
            tool->Delay(10);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 1250);								//馬達x值不動
            tool->Delay(10);
        }
        else
        {
            int move_vertical = (SprintInfo->center_y - 120);
            int move_horizontal = (SprintInfo->center_x - 160);
            int tmpangle_y = 35.0 * (double)move_vertical / ImageLength; 			//changed from 40 to 24
            int tmpangle_x = 50.0 * (double)move_horizontal / ImageWidth;			//changed from 30 to 50
            if (abs(tmpangle_y) > 2 || abs(tmpangle_x) > 6)
            {
                SprintInfo->head_motor_y -= 11.378 * tmpangle_y * 0.4;
                if (SprintInfo->head_motor_y > 2210)								//2047 + 40*4+3 old:551
                {
                    SprintInfo->head_motor_y = 2210;
                }
                else if (SprintInfo->head_motor_y < 1100)							//2047 - 400*4+3 old:111
                {
                    SprintInfo->head_motor_y = 1100;
                }
                SprintInfo->head_motor_x -= 11.378 * tmpangle_x * 0.4;
                tool->Delay(40);
                if (SprintInfo->head_motor_x > 2250)								//馬達轉動的界限 2047 + 50*4+3 old:561
                {
                    SprintInfo->head_motor_x = 2250;
                }
                else if (SprintInfo->head_motor_x < 1844)							//馬達轉動的界限 2047 - 50*4+3 old:461
                {
                    SprintInfo->head_motor_x = 1844;
                }
            }
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 1000);			//將馬達y值改為上述head_motor_y值 old_speed:211
            tool->Delay(10);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, SprintInfo->head_motor_x, 1500);			//將馬達x值改為上述head_motor_x值 old_speed:211
            tool->Delay(10);
        }
        ROS_INFO(" head_angle_x = %d", SprintInfo->head_motor_x);
    }
}
/****************************************************************************************************/
void KidsizeStrategy::initial_strategy(void) //bool
{
    SprintInfo->head_motor_y = 2047;												//設定頭部馬達初始角度 2047 - 36*4+3 old:476
    SprintInfo->head_motor_x = 2047;												//設定頭部馬達初始角度 2047 		 old:511
    SprintInfo->fram_count = 0;
    
    ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 2000);					//old_speed:211
    tool->Delay(10);
    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, SprintInfo->head_motor_x, 1250);					//old_speed:211
    tool->Delay(10);
    
    SprintInfo->SpintInfomation->SprForWard = true;        							//設定為前進
    SprintInfo->SpintInfomation->send_x = forward_initial_x; 						//給予初始x
    SprintInfo->SpintInfomation->send_y = Initial_Y;
    SprintInfo->SpintInfomation->tmp_theta = 0;
    SprintInfo->SpintInfomation->send_theta = 0;

    SprintInfo->SpintInfomation->checkback_cnt = 0;
    ros_com->drawImageFunction(1, DrawMode::DrawObject, 0, 0, 0, 0, 255, 255, 0);
    ros_com->drawImageFunction(2, DrawMode::DrawObject, 0, 0, 0, 0, 255, 255, 0);
/*******************************************aruco***************************************************/
    aruco_direction = 0;
    aruco_head = 0;
    aruco_distence = 0;
    get_arucoimage = false;
}
/****************************************************************************************************/
void KidsizeStrategy::do_sprint_forward_part(int left_pixel, int right_pixel, int left_turn_value, int right_turn_value)
{

    if (SprintInfo->center_x > right_pixel)
    {
        SprintInfo->SpintInfomation->tmp_theta += right_turn_value;					//右轉
        ROS_INFO("forward right turn!");
    }
    else if (SprintInfo->center_x < left_pixel)
    {
        SprintInfo->SpintInfomation->tmp_theta += left_turn_value;					//左轉
        ROS_INFO("forward left turn!");
    }
}
/****************************************************************************************************/
void KidsizeStrategy::do_forward(void)
{
    
    if(total_size >= without_left)
    {
        if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_left_three;
                printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
            }
        else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_right_three;
                printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
            }
        else
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta;
                SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
            }
    }
    else
    {
        if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_left_three;
                printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
            }
        else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_right_three;
                printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
            }
        else
            {
                SprintInfo->SpintInfomation->send_theta = forward_initial_theta;
                SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
            }   
    }
    /*
    ROS_INFO("~~~~~~do_forward information start~~~~~~");
    if (total_size < REDSIZE_50)
    {
        //ROS_INFO("50cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_one, forward_theta_right_one);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((total_size >= REDSIZE_50) && (total_size < REDSIZE_100))
    {
        //ROS_INFO("50cm~100cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_one, forward_theta_right_one);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((total_size >= REDSIZE_100) && (total_size < REDSIZE_150))
    {
        //ROS_INFO("100cm~150cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_two, forward_theta_right_two);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((total_size >= REDSIZE_150) && (total_size < REDSIZE_200))
    {
        //ROS_INFO("150cm~200cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_two, forward_theta_right_two);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((total_size >= REDSIZE_200) && (total_size < REDSIZE_250))
    {
        //ROS_INFO("200cm~250cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_three, forward_theta_right_three);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((total_size >= REDSIZE_250) && (total_size < REDSIZE_300))
    {
        //ROS_INFO("250cm~300cm");
        do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_three, forward_theta_right_three);
        SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_250);
    }
    else if (total_size >= REDSIZE_300)
    {
		//ROS_INFO("300cm~");
		if(change_mode == 0)														//yb change_mode == 0
		{
		    do_sprint_forward_part(forward_left_pixel, forward_right_pixel, forward_theta_left_three, forward_theta_right_three);
		    SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
		}backward_x_add
		else																		//without watching ball change_mode == 1
		{
			SprintInfo->SpintInfomation->tmp_theta = forward_initial_theta;
		    SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
		}
    }
    ROS_INFO("~~~~~~do_forward information end~~~~~~");
    */
}
/****************************************************************************************************/
void KidsizeStrategy::do_sprint_backward_part_yb(double left_dis, double right_dis, int left_turn_value, int right_turn_value)
{
    if (YBrat < left_dis)
	{
	    SprintInfo->SpintInfomation->tmp_theta += left_turn_value;
	    check_headerror = false;
	    ROS_INFO("~~~~~~~yb backward leftback information start~~~~~~~~");		            //左轉後退  向軌道內側轉	左肩向後
	    ROS_INFO("tmp_theta = %d",SprintInfo->SpintInfomation->tmp_theta);
	    ROS_INFO("fram_count = %2d",SprintInfo->fram_count);
	    ROS_INFO("~~~~~~~yb backward leftback information end~~~~~~~~");		            //左轉後退  向軌道內側轉	左肩向後
    }
    else if (YBrat > right_dis)
    {
        SprintInfo->SpintInfomation->tmp_theta += right_turn_value;
        check_headerror = false;
        ROS_INFO("~~~~~~~yb backward rightback information start~~~~~~~~");		            //右轉後退	右肩向後
        ROS_INFO("tmp_theta = %d",SprintInfo->SpintInfomation->tmp_theta);
        ROS_INFO("fram_count = %2d",SprintInfo->fram_count);
        ROS_INFO("~~~~~~~yb backward rightback information end~~~~~~~~");		            //右轉後退	右肩向後
    }
    else if(check_headerror == true && change_mode == 0)							        //後退直走 use to face on target
    {
		int tmp_error = SprintInfo->head_motor_x - 2047;							        //馬達刻度
		if (YBrat > right_dis - 0.05 || tmp_error > (HEADERROR) )							//tmp_error > (HEADERROR)	左轉
		{
			SprintInfo->SpintInfomation->tmp_theta = SprintInfo->SpintInfomation->tmp_theta + (int)(tmp_error - HEADERROR) / 40;
			SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
			ROS_INFO("~~~~~~~right shoulder back(head error) information start~~~~~~~~");	//左轉後退 左肩向後
			ROS_INFO("tmp_theta = %d",SprintInfo->SpintInfomation->tmp_theta);
			ROS_INFO("head_motor_x = %d",SprintInfo->head_motor_x);
            ROS_INFO("headerror YBrate = %1.5f",YBrat);
			ROS_INFO("~~~~~~~right shoulder back(head error) information end~~~~~~~~");	//左轉後退 左肩向後
		}
		else if (YBrat < left_dis + 0.05 || tmp_error < -(HEADERROR))						//tmp_error < -(HEADERROR)	右轉
		{
			SprintInfo->SpintInfomation->tmp_theta = SprintInfo->SpintInfomation->tmp_theta + (int)(tmp_error + HEADERROR) / 40;
			SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
			ROS_INFO("~~~~~~~left shoulder back(head error) information start~~~~~~~~");	//右轉後退 右肩向後
			ROS_INFO("tmp_theta = %d",SprintInfo->SpintInfomation->tmp_theta);
			ROS_INFO("head_motor_x = %d",SprintInfo->head_motor_x);
            ROS_INFO("headerror YBrate = %1.5f",YBrat);
			ROS_INFO("~~~~~~~left shoulder back(head error) information end~~~~~~~~");	//右轉後退 右肩向後
		}
	}
	else
	{
	    ROS_INFO("~~~~~~~~~~~~~fram_count = %2d do nothing~~~~~~~~~~~~~",SprintInfo->fram_count);
	}
}
/****************************************************************************************************/
void KidsizeStrategy::do_backward_without_watch_ball(int left_turn_value, int right_turn_value)
{
    ROS_INFO("~~~~~~do_backward_without_watch_ball information start~~~~~~");
    if(timeuse <= right_timeuse)													//後退右轉
    {
		gettimeofday(&tend, NULL);
        timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
		SprintInfo->SpintInfomation->send_theta = backward_initial_theta + right_turn_value;
		ROS_INFO("timeuse = %5.4f",timeuse);
		ROS_INFO("right_turn send_theta = %d", SprintInfo->SpintInfomation->send_theta);
    }
    else if(timeuse > right_timeuse && timeuse <= (right_timeuse+initial_timeuse) )	//後退直走
    {
		gettimeofday(&tend, NULL);
       	timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
		SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
		ROS_INFO("timeuse = %5.4f",timeuse);
		ROS_INFO("center send_theta = %d", SprintInfo->SpintInfomation->send_theta);
    } 
    else if(timeuse > (right_timeuse+initial_timeuse) && timeuse <= (right_timeuse+initial_timeuse+left_timeuse) )
    {																				
		gettimeofday(&tend, NULL);													//後退左轉
       	timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
		SprintInfo->SpintInfomation->send_theta = backward_initial_theta + left_turn_value;
		ROS_INFO("timeuse = %5.4f",timeuse);
		ROS_INFO("left_turn send_theta = %d", SprintInfo->SpintInfomation->send_theta);
    } 
    else 
    {
		gettimeofday(&tstart, NULL);
		timeuse = 0;
		ROS_INFO("initial_timeuse = %f",timeuse);
    }
    SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
    ROS_INFO("~~~~~~do_backward_without_watch_ball information end~~~~~~");
}
/****************************************************************************************************/
void KidsizeStrategy::do_backward_ybrat()
{
    if(total_size >= without_right)
    {
        if(SprintInfo->head_motor_x == 1844 || SprintInfo->head_motor_x == 2250)
        {
            if(SprintInfo->head_motor_x == 1844)
            {
                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_left_two;
                printf("\n1844 danger zone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
            }
            else
            {
                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_right_two;
                printf("\n2250 danger zone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
            }
        }
        else
        {
            if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
            {
                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_left_three;
                printf("\nturn backward_theta_left_three=%d\n",backward_theta_left_three);
                printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
            }
            else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
            {
                SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_right_three;
                printf("\nturn backward_theta_left_three=%d\n",backward_theta_right_three);
                printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
            }
            else
            {
                SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
                SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
            }
        }
    }
    else
    {
        if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
        {
            SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_left_three;
            printf("\nturn backward_theta_left_three=%d\n",backward_theta_left_three);
            printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
            SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
        }
        else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
        {
            SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_right_three;
            printf("\nturn backward_theta_left_three=%d\n",backward_theta_right_three);
            printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
            SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
        }
        else
        {
            SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
            SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
        }
    }
    
    /*
    ROS_INFO("~~~~~~~~just backward ybrate start~~~~~~~~~");
    if (total_size < REDSIZE_50)
    {
		do_sprint_backward_part_yb(SprintInfo->backward_50L, SprintInfo->backward_50R, backward_theta_left_one, backward_theta_right_one);
        //ROS_INFO("back_before_50");
    }
    else if ((total_size > REDSIZE_50) && (total_size <= REDSIZE_100))
    {
        do_sprint_backward_part_yb(SprintInfo->backward_100L, SprintInfo->backward_100R, backward_theta_left_one, backward_theta_right_one);
        //ROS_INFO("back_between_50~100 ");
    }
    else if ((total_size >= REDSIZE_100) && (total_size < REDSIZE_150))
    {
        do_sprint_backward_part_yb(SprintInfo->backward_150L, SprintInfo->backward_150R, backward_theta_left_two, backward_theta_right_two);
        //ROS_INFO("back_between_100~150 ");
    }
    else if ((total_size >= REDSIZE_150) && (total_size < REDSIZE_200))
    {
        do_sprint_backward_part_yb(SprintInfo->backward_200L, SprintInfo->backward_200R, backward_theta_left_two, backward_theta_right_two);
        //ROS_INFO("back_between_150~200 ");
    }
    else if ((total_size >= REDSIZE_200) && (total_size < REDSIZE_250))
    {
        do_sprint_backward_part_yb(SprintInfo->backward_250L, SprintInfo->backward_250R, backward_theta_left_three, backward_theta_right_three);
        //ROS_INFO("back_between_200~250 ");
    }
    else if ((total_size >= REDSIZE_250) && (total_size < REDSIZE_300))
    {
        //250cm~300cm
        do_sprint_backward_part_yb(SprintInfo->backward_300L, SprintInfo->backward_300R, backward_theta_left_three, backward_theta_right_three);
        //ROS_INFO("b 250~300 send_x ");
    }
    else if (total_size >= REDSIZE_300)
    {
        do_sprint_backward_part_yb(SprintInfo->backward_310L, SprintInfo->backward_310R, backward_theta_left_three, backward_theta_right_three);
        //ROS_INFO("back_between_300~");
    }
	SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);	//backward speed limit
    ROS_INFO("~~~~~~~~just backward ybrate end~~~~~~~~~");
    */
}
/****************************************************************************************************/
void KidsizeStrategy::initparameterpath(void)
{
	while(tool->parameterPath == "N")
	{
		tool->parameterPath = tool->getPackagePath("strategy");
	}
	printf("tool->parameterPath is %s\n", tool->parameterPath.c_str());
}
/****************************************************************************************************/
void KidsizeStrategy::load_strategy_parameter_ini(void)
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, tool->parameterPath.c_str());														//copy string
    strcat(path, "/quickchage.ini");												//Connected to path
    fin.open(path, ios::in);
    //fin.open(("/home/iclab/Desktop/SP/src/strategy/Parameter/quickchage.ini"), ios::in);
    try
    {
		SprintInfo->backward_310L = tool->readvalue(fin, "backward_310L", 1);
		SprintInfo->backward_310R = tool->readvalue(fin, "backward_310R", 1);
		SprintInfo->backward_300L = tool->readvalue(fin, "backward_300L", 1);
		SprintInfo->backward_300R = tool->readvalue(fin, "backward_300R", 1);
		SprintInfo->backward_250L = tool->readvalue(fin, "backward_250L", 1);
		SprintInfo->backward_250R = tool->readvalue(fin, "backward_250R", 1);
		SprintInfo->backward_200L = tool->readvalue(fin, "backward_200L", 1);
		SprintInfo->backward_200R = tool->readvalue(fin, "backward_200R", 1);
		SprintInfo->backward_150L = tool->readvalue(fin, "backward_150L", 1);
		SprintInfo->backward_150R = tool->readvalue(fin, "backward_150R", 1);
		SprintInfo->backward_100L = tool->readvalue(fin, "backward_100L", 1);
		SprintInfo->backward_100R = tool->readvalue(fin, "backward_100R", 1);
		SprintInfo->backward_50L = tool->readvalue(fin, "backward_50L", 1);
		SprintInfo->backward_50R = tool->readvalue(fin, "backward_50R", 1);

		forward_x_add = tool->readvalue(fin, "forward_x_add", 0);
		forward_x_max = tool->readvalue(fin, "forward_x_max", 0);
		backward_x_add = tool->readvalue(fin, "backward_x_add", 0);
        backward_x_max = tool->readvalue(fin, "backward_x_max", 0);
		forward_initial_x = tool->readvalue(fin, "forward_initial_x", 0);
        backward_initial_x = tool->readvalue(fin,"backward_initial_x", 0);

        slowdown_250 = tool->readvalue(fin, "slowdown_250", 0);
        slowdown_300 = tool->readvalue(fin, "slowdown_300", 0);
		slowdown_initial_x = tool->readvalue(fin, "slowdown_initial_x", 0);

        FRAMCOUNT = tool->readvalue(fin, "FRAMCOUNT", 0);
		change_mode = tool ->readvalue(fin, "change_mode", 0);
        back_total_size = tool->readvalue(fin, "back_total_size", 0);

        forward_initial_theta = tool->readvalue(fin, "forward_initial_theta", 0);
        forward_theta_left_one = tool->readvalue(fin, "forward_theta_left_one", 0);
        forward_theta_right_one = tool->readvalue(fin, "forward_theta_right_one", 0);
        forward_theta_left_two = tool->readvalue(fin, "forward_theta_left_two", 0);
        forward_theta_right_two = tool->readvalue(fin, "forward_theta_right_two", 0);
        forward_theta_left_three = tool->readvalue(fin, "forward_theta_left_three", 0);
        forward_theta_right_three = tool->readvalue(fin, "forward_theta_right_three", 0);

        backward_initial_theta = tool->readvalue(fin, "backward_initial_theta", 0);
		backward_theta_left_one = tool->readvalue(fin, "backward_theta_left_one", 0);
        backward_theta_right_one = tool->readvalue(fin, "backward_theta_right_one", 0);
        backward_theta_left_two = tool->readvalue(fin, "backward_theta_left_two", 0);
        backward_theta_right_two = tool->readvalue(fin, "backward_theta_right_two", 0);
        backward_theta_left_three = tool->readvalue(fin, "backward_theta_left_three", 0);
        backward_theta_right_three = tool->readvalue(fin, "backward_theta_right_three", 0);

		delay_ybtime = tool->readvalue(fin, "delay_ybtime", 0);
		without_left = tool->readvalue(fin, "without_left", 0);
        without_right = tool->readvalue(fin, "without_right", 0);
		right_timeuse = tool->readvalue(fin, "right_timeuse", 0);
		initial_timeuse = tool->readvalue(fin, "initial_timeuse", 0);
		left_timeuse = tool->readvalue(fin, "left_timeuse", 0);
        //IMU_angle_range = tool->readvalue(fin,"IMU_angle_range", 0);

    }
    catch (exception e)
    {
		ROS_INFO("READ ERROR");
    }
}
/****************************************************************************************************/
void KidsizeStrategy::SaveParameter(const strategy::quickchange &msg)    			//网页
{
    ROS_INFO("Enter");
    char path[200];
    strcpy(path, tool->parameterPath.c_str());
    strcat(path, "/quickchage.ini");
    ofstream OutFile(path);

    ROS_INFO("SaveParameter");
    OutFile << "backward_310L = ";
    OutFile << msg.backward_310L;
    OutFile << "\n";

    OutFile << "backward_310R = ";
    OutFile << msg.backward_310R;
    OutFile << "\n";

    OutFile << "backward_300L = ";
    OutFile << msg.backward_300L;
    OutFile << "\n";

    OutFile << "backward_300R = ";
    OutFile << msg.backward_300R;
    OutFile << "\n";

    OutFile << "backward_250L = ";
    OutFile << msg.backward_250L;
    OutFile << "\n";

    OutFile << "backward_250R = ";
    OutFile << msg.backward_250R;
    OutFile << "\n";

    OutFile << "backward_200L = ";
    OutFile << msg.backward_200L;
    OutFile << "\n";

    OutFile << "backward_200R = ";
    OutFile << msg.backward_200R;
    OutFile << "\n";

    OutFile << "backward_150L = ";
    OutFile << msg.backward_150L;
    OutFile << "\n";

    OutFile << "backward_150R = ";
    OutFile << msg.backward_150R;
    OutFile << "\n";

    OutFile << "backward_100L = ";
    OutFile << msg.backward_100L;
    OutFile << "\n";

    OutFile << "backward_100R = ";
    OutFile << msg.backward_100R;
    OutFile << "\n";

    OutFile << "backward_50L = ";
    OutFile << msg.backward_50L;
    OutFile << "\n";

    OutFile << "backward_50R = ";
    OutFile << msg.backward_50R;
    OutFile << "\n";

    OutFile << "forward_x_add = ";
    OutFile << msg.forward_x_add;
    OutFile << "\n";

    OutFile << "forward_x_max = ";
    OutFile << msg.forward_x_max;
    OutFile << "\n";

    OutFile << "backward_x_add = ";
    OutFile << msg.backward_x_add;
    OutFile << "\n";

    OutFile << "backward_x_max = ";
    OutFile << msg.backward_x_max;
    OutFile << "\n";

    OutFile << "forward_initial_x = ";
    OutFile << msg.forward_initial_x;
    OutFile << "\n";

    OutFile << "backward_initial_x = ";
    OutFile << msg.backward_initial_x;
    OutFile << "\n";

    OutFile << "slowdown_250 = ";
    OutFile << msg.slowdown_250;
    OutFile << "\n";

    OutFile << "slowdown_300 = ";
    OutFile << msg.slowdown_300;
    OutFile << "\n";

    OutFile << "slowdown_initial_x = ";
    OutFile << msg.slowdown_initial_x;
    OutFile << "\n";

    OutFile << "FRAMCOUNT = ";
    OutFile << msg.FRAMCOUNT;
    OutFile << "\n";

    OutFile << "change_mode = ";
    OutFile << msg.change_mode;
    OutFile << "\n";

    OutFile << "back_total_size = ";
    OutFile << msg.back_total_size;
    OutFile << "\n";

    OutFile << "forward_initial_theta = ";
    OutFile << msg.forward_initial_theta;
    OutFile << "\n";

    OutFile << "forward_theta_left_one = ";
    OutFile << msg.forward_theta_left_one;
    OutFile << "\n";

    OutFile << "forward_theta_right_one = ";
    OutFile << msg.forward_theta_right_one;
    OutFile << "\n";

    OutFile << "forward_theta_left_two = ";
    OutFile << msg.forward_theta_left_two;
    OutFile << "\n";

    OutFile << "forward_theta_right_two = ";
    OutFile << msg.forward_theta_right_two;
    OutFile << "\n";

    OutFile << "forward_theta_left_three = ";
    OutFile << msg.forward_theta_left_three;
    OutFile << "\n";

    OutFile << "forward_theta_right_three = ";
    OutFile << msg.forward_theta_right_three;
    OutFile << "\n";

    OutFile << "backward_initial_theta = ";
    OutFile << msg.backward_initial_theta;
    OutFile << "\n";

    OutFile << "backward_theta_left_one = ";
    OutFile << msg.backward_theta_left_one;
    OutFile << "\n";

    OutFile << "backward_theta_right_one = ";
    OutFile << msg.backward_theta_right_one;
    OutFile << "\n";

    OutFile << "backward_theta_left_two = ";
    OutFile << msg.backward_theta_left_two;
    OutFile << "\n";

    OutFile << "backward_theta_right_two = ";
    OutFile << msg.backward_theta_right_two;
    OutFile << "\n";

    OutFile << "backward_theta_left_three = ";
    OutFile << msg.backward_theta_left_three;
    OutFile << "\n";

    OutFile << "backward_theta_right_three = ";
    OutFile << msg.backward_theta_right_three;
    OutFile << "\n";

    OutFile << "delay_ybtime = ";
    OutFile << msg.delay_ybtime;
    OutFile << "\n";

    OutFile << "without_left = ";
    OutFile << msg.without_left;
    OutFile << "\n";

    OutFile << "without_right = ";
    OutFile << msg.without_right;
    OutFile << "\n";

    OutFile << "right_timeuse = ";
    OutFile << msg.right_timeuse;
    OutFile << "\n";

    OutFile << "initial_timeuse = ";
    OutFile << msg.initial_timeuse;
    OutFile << "\n";

    OutFile << "left_timeuse = ";
    OutFile << msg.left_timeuse;
    OutFile << "\n";

    OutFile << "IMU_angle_range = ";
    OutFile << msg.IMU_angle_range;
    OutFile << "\n";

    OutFile.close();
    ROS_INFO("SaveParameter_END");
}
/****************************************************************************************************/
bool KidsizeStrategy::load_strategy_parameter(strategy::Loadquickchange::Request &Parameter_req, strategy::Loadquickchange::Response &Parameter_res)
{
    fstream fin;
    string sTmp;
    ROS_INFO("LoadSP");
    char line[100];
    char path[200];
    strcpy(path, tool->parameterPath.c_str());														//copy string
    strcat(path, "/quickchage.ini");												//Connected to path
    fin.open(path, ios::in);
    //fin.open(("/home/iclab/Desktop/SP/src/strategy/Parameter/quickchage.ini"), ios::in);
    try
    {
        ROS_INFO("Start_Load");
        Parameter_res.backward_310L = tool->readvalue(fin, "backward_310L", 1);

        Parameter_res.backward_310R = tool->readvalue(fin, "backward_310R", 1);

        Parameter_res.backward_300L = tool->readvalue(fin, "backward_300L", 1);
        Parameter_res.backward_300R = tool->readvalue(fin, "backward_300R", 1);

        Parameter_res.backward_250L = tool->readvalue(fin, "backward_250L", 1);
        Parameter_res.backward_250R = tool->readvalue(fin, "backward_250R", 1);

        Parameter_res.backward_200L = tool->readvalue(fin, "backward_200L", 1);
        Parameter_res.backward_200R = tool->readvalue(fin, "backward_200R", 1);

        Parameter_res.backward_150L = tool->readvalue(fin, "backward_150L", 1);
        Parameter_res.backward_150R = tool->readvalue(fin, "backward_150R", 1);

        Parameter_res.backward_100L = tool->readvalue(fin, "backward_100L", 1);
        Parameter_res.backward_100R = tool->readvalue(fin, "backward_100R", 1);

        Parameter_res.backward_50L = tool->readvalue(fin, "backward_50L", 1);
        Parameter_res.backward_50R = tool->readvalue(fin, "backward_50R", 1);

        Parameter_res.forward_x_add = tool->readvalue(fin, "forward_x_add", 0);
        Parameter_res.forward_x_max = tool->readvalue(fin, "forward_x_max", 0);

        Parameter_res.backward_x_add = tool->readvalue(fin, "backward_x_add", 0);
        Parameter_res.backward_x_max = tool->readvalue(fin, "backward_x_max", 0);

        Parameter_res.forward_initial_x = tool->readvalue(fin, "forward_initial_x", 0);
        Parameter_res.backward_initial_x = tool->readvalue(fin,"backward_initial_x", 0);


        Parameter_res.slowdown_250 = tool->readvalue(fin, "slowdown_250", 0);
        Parameter_res.slowdown_300 = tool->readvalue(fin, "slowdown_300", 0);
		Parameter_res.slowdown_initial_x = tool->readvalue(fin, "slowdown_initial_x", 0);

        Parameter_res.FRAMCOUNT = tool->readvalue(fin, "FRAMCOUNT", 0);
		Parameter_res.change_mode = tool ->readvalue(fin, "change_mode", 0);

        Parameter_res.back_total_size = tool->readvalue(fin, "back_total_size", 0);

        Parameter_res.forward_initial_theta = tool->readvalue(fin, "forward_initial_theta", 0);

        Parameter_res.forward_theta_left_one = tool->readvalue(fin, "forward_theta_left_one", 0);
        Parameter_res.forward_theta_right_one = tool->readvalue(fin, "forward_theta_right_one", 0);

        Parameter_res.forward_theta_left_two = tool->readvalue(fin, "forward_theta_left_two", 0);
        Parameter_res.forward_theta_right_two = tool->readvalue(fin, "forward_theta_right_two", 0);

        Parameter_res.forward_theta_left_three = tool->readvalue(fin, "forward_theta_left_three", 0);
        Parameter_res.forward_theta_right_three = tool->readvalue(fin, "forward_theta_right_three", 0);

        Parameter_res.backward_initial_theta = tool->readvalue(fin, "backward_initial_theta", 0);

        Parameter_res.backward_theta_left_one = tool->readvalue(fin, "backward_theta_left_one", 0);
        Parameter_res.backward_theta_right_one = tool->readvalue(fin, "backward_theta_right_one", 0);

        Parameter_res.backward_theta_left_two = tool->readvalue(fin, "backward_theta_left_two", 0);
        Parameter_res.backward_theta_right_two = tool->readvalue(fin, "backward_theta_right_two", 0);

        Parameter_res.backward_theta_left_three = tool->readvalue(fin, "backward_theta_left_three", 0);
        Parameter_res.backward_theta_right_three = tool->readvalue(fin, "backward_theta_right_three", 0);

		Parameter_res.delay_ybtime = tool->readvalue(fin, "delay_ybtime", 0);
		Parameter_res.without_left = tool->readvalue(fin, "without_left", 0);
        Parameter_res.without_right = tool->readvalue(fin, "without_right", 0);
		Parameter_res.right_timeuse = tool->readvalue(fin, "right_timeuse", 0);
		Parameter_res.initial_timeuse = tool->readvalue(fin, "initial_timeuse", 0);
		Parameter_res.left_timeuse = tool->readvalue(fin, "left_timeuse", 0);
        //Parameter_res.IMU_angle_range = tool->readvalue(fin, "IMU_angle_range", 0);

    }
    catch (exception e)
    {
		ROS_INFO("READ ERROR");
    }
    	ROS_INFO("LoadSP_successful");
    return true;
}
/****************************************************************************************************/
void KidsizeStrategy::GetArucoMatrix(const strategy::aruco_matrix &msg)
{
    aruco_direction = msg.tvecs[0]*100;
    aruco_head = msg.tvecs[1]*100;
    aruco_distence = msg.tvecs[2]*1000;
    get_arucoimage = msg.get_arucoimage;
	if(strategy_info->getStrategyStart())
	{
	//ROS_INFO("tvecs0 = %.4f,aruco_direction = %.2f",msg.tvecs[0],abs(aruco_direction));
	//ROS_INFO("tvecs1 = %.4f,aruco_head      = %.2f",msg.tvecs[1],abs(aruco_head));
	//ROS_INFO("tvecs2 =  %.4f,aruco_distence  = %.2f",msg.tvecs[2],abs(aruco_distence));
	}
}
/****************************************************************************************************/
void KidsizeStrategy::aruco_head_strategy(void)
{			
    if (get_arucoimage)
    {
        if (SprintInfo->SpintInfomation->SprForWard)
        {
            if(aruco_distence >= ARUCODISTENCE_200)
            {
                aruco_headmove = abs(abs(aruco_head) - 15);
                aruco_headangle = aruco_headmove * 0.15;
			    //ROS_INFO("aruco_headmove = %d",aruco_headmove);
			    //ROS_INFO("aruco_headangle = %d",aruco_headangle);
                tool->Delay(80);
                if (aruco_headangle > 0)
                {
                    SprintInfo->head_motor_y -= 10.378 * aruco_headangle;
                    if (SprintInfo->head_motor_y > 2200)
                    {
                    SprintInfo->head_motor_y = 2200;
                    }
                    else if (SprintInfo->head_motor_y < 1800)
                    {
                        SprintInfo->head_motor_y = 1800;
                    }
                }
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 1000);			//將馬達y值改為上述head_motor_y值 old_speed:211
                tool->Delay(10);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 1250);								//馬達x值不動
                tool->Delay(10);
            }
            else
            {
                aruco_headmove = abs(abs(aruco_head) - 15);
                aruco_headangle = aruco_headmove * 0.15;
			    //ROS_INFO("aruco_headmove = %d",aruco_headmove);
			    //ROS_INFO("aruco_headangle = %d",aruco_headangle);
                tool->Delay(80);
                if (aruco_headangle > 0)
                {
                    SprintInfo->head_motor_y -= 16.378 * aruco_headangle;
                    if (SprintInfo->head_motor_y > 1800)
                    {
                    SprintInfo->head_motor_y = 1800;
                    }
                    else if (SprintInfo->head_motor_y < 1400)
                    {
                        SprintInfo->head_motor_y = 1400;
                    }
                }
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 2000);			//將馬達y值改為上述head_motor_y值 old_speed:211
                tool->Delay(10);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 1250);								//馬達x值不動
                tool->Delay(10);

            }

        }
        else
        {	
	    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2200, 1000);			//將馬達y值改為上述head_motor_y值 old_speed:211
            tool->Delay(10);
            if(aruco_distence <= ARUCODISTENCE_300)
            {
                aruco_headangle_limit = 2200;
            }
            if(aruco_distence <= ARUCODISTENCE_250 && aruco_distence > ARUCODISTENCE_300)
            {
                aruco_headangle_limit = 2200;
            }			
            if(aruco_distence <= ARUCODISTENCE_200 && aruco_distence > ARUCODISTENCE_250)
            {
                aruco_headangle_limit = 2200;
            }
            if(aruco_distence <= ARUCODISTENCE_150 && aruco_distence > ARUCODISTENCE_200)
            {
                aruco_headangle_limit = 2200;
            }
            if(aruco_distence <= ARUCODISTENCE_100 && aruco_distence > ARUCODISTENCE_150)
            {
                aruco_headangle_limit = 2200;
            }
            if(aruco_distence <= ARUCODISTENCE_50 && aruco_distence > ARUCODISTENCE_100)
            {
                aruco_headangle_limit = 2200;
            }
            if(aruco_distence > ARUCODISTENCE_50)
            {
                aruco_headangle_limit = 2200;
            }
            /*aruco_headmove = abs(abs(aruco_head) - 15);
            aruco_headangle = aruco_headmove * 0.2;
            if (aruco_headangle > 0)
            {
                SprintInfo->head_motor_y += 2.378 * aruco_headangle;
                if (SprintInfo->head_motor_y > 2000)
                {
                    SprintInfo->head_motor_y = 2000;
                }
                else if (SprintInfo->head_motor_y < 1400)
                {
                    SprintInfo->head_motor_y = 1400;
                }
            }*/
            /*ros_com->sendHeadMotor(HeadMotorID::VerticalID, SprintInfo->head_motor_y, 1000);			//將馬達y值改為上述head_motor_y值 old_speed:211
            tool->Delay(10);*/
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 1250);								//馬達x值不動
            tool->Delay(10);
        }
    }
}
void KidsizeStrategy::aruco_move_strategy(void)
{	
	
	if (get_arucoimage)              
                                                                                       //if mode==1 and back it will be true
	{
		if (SprintInfo->SpintInfomation->SprForWard)                                                                    //是否前進中
		{
			aruco_do_forward();
			tao=p;
			if (aruco_distence < aruco_back_distence)                                                                 //判斷是否後退走
			{
				ROS_INFO("~~~~~~aruco turn Backward~~~~~~");
				SprintInfo->SpintInfomation->checkback_cnt++;
				SprintInfo->SpintInfomation->send_x = slowdown_initial_x;                                               //步態X軸參數
				
			}
			else
			{
				SprintInfo->SpintInfomation->checkback_cnt = 0;
			}
			if (SprintInfo->SpintInfomation->checkback_cnt > CHECKBACK_CNT)                                             //如回程條件成立
			{
				SprintInfo->SpintInfomation->SprForWard = false;                                                        //close forward
				SprintInfo->SpintInfomation->tmp_theta = backward_initial_theta;
				SprintInfo->SpintInfomation->send_theta = backward_initial_theta;
				SprintInfo->SpintInfomation->send_x = backward_initial_x;                                               //步態X軸參數
			}
		}
		else
		{
			ROS_INFO("aruco Do Backward!!!!!!");
            aruco_do_backward();
		}
	}
	else                                                                                                          //沒抓到目標
	{
		ROS_INFO("aruco lost target!\n");
		if (SprintInfo->SpintInfomation->SprForWard && tao==0)
		{
			for(jing=0;jing<=2;jing++)
		{
			int aa=100;
			SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + aa); 
			tool->Delay(150);
			
		}
			
		    if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
                {
                    
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_left_one;                              
                        printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
	    	    else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
                {
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_right_one;                            
                    printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
 		}
    		else if(SprintInfo->SpintInfomation->SprForWard && tao==1)
		{   		
		    for(jing=0;jing<=2;jing++)
		{
			int aa=100;
			SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + aa); 	
			tool->Delay(150);
			
		}
		
		    if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
                {
                    
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_left_one;                              
                        printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
	    	    else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
                {
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_right_one;                            
                    printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
		}
		else if(SprintInfo->SpintInfomation->SprForWard && tao==2)
		{
		 if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
                {
                    
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_left_one;                              
                        printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
	    	    else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
                {
                    SprintInfo->SpintInfomation->send_theta = forward_initial_theta + forward_theta_right_one;                            
                    printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
		}
		else
		{
              	   for(jing=0;jing<=2;jing++)
		{
			int aa=100;
			SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x - aa); 	
			//printf("\nsend_x=%d\n",SprintInfo->SpintInfomation->send_x);
			tool->Delay(200);
			
		}
		 if(SprintInfo->IMU_now <= SprintInfo->IMU_right)
                {
                    
                    SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_left_three;                              
                        printf("\nturn right,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
	    	    else if(SprintInfo->IMU_now >= SprintInfo->IMU_left)
                {
                    SprintInfo->SpintInfomation->send_theta = backward_initial_theta + backward_theta_right_three;                            
                    printf("\nturn left,send_theta=%d\n",SprintInfo->SpintInfomation->send_theta);
                }
                else
                {
			        SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);
                }
        }
	}
	
	if (SprintInfo->SpintInfomation->SprForWard)
	{
		    SprintInfo->SpintInfomation->send_theta += SprintInfo->SpintInfomation->tmp_theta;
	    	SprintInfo->SpintInfomation->send_theta = min(Forward_ThetaMax + forward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn left Limit
	    	SprintInfo->SpintInfomation->send_theta = max(Forward_ThetaMin + forward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn right Limit
	    	SprintInfo->SpintInfomation->tmp_theta = forward_initial_theta;	//initializ
	}
	else
	{
        if(get_arucoimage)
        {
	    	//ROS_INFO("~~~~~~~~~~~~~aruco you send information start~~~~~~~~~~~~~");
	    	//ROS_INFO("send_theta = %2d",SprintInfo->SpintInfomation->send_theta);
	    	//ROS_INFO("tmp_theta = %2d",SprintInfo->SpintInfomation->tmp_theta);
	    	SprintInfo->SpintInfomation->send_theta += (int)SprintInfo->SpintInfomation->tmp_theta;
	    	//ROS_INFO("send_theta = %2d",SprintInfo->SpintInfomation->send_theta);
	    	//ROS_INFO("~~~~~~~~~~~~~aruco you send information end~~~~~~~~~~~~~");
	    	SprintInfo->SpintInfomation->send_theta = min(Backward_ThetaMax + backward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn left Limit
	    	SprintInfo->SpintInfomation->send_theta = max(Backward_ThetaMin + backward_initial_theta, SprintInfo->SpintInfomation->send_theta);	//turn right Limit
	    	SprintInfo->SpintInfomation->tmp_theta = backward_initial_theta;
	    	tool->Delay(50);
        }
	}
	
    //ROS_INFO("send_theta = %d", SprintInfo->SpintInfomation->send_theta);
	//ROS_INFO("speed = %4d", SprintInfo->SpintInfomation->send_x);
    if (SprintInfo->SpintInfomation->SprForWard)
	{
	    ros_com->sendContinuousValue(SprintInfo->SpintInfomation->send_x, SprintInfo->SpintInfomation->send_y, 0, SprintInfo->SpintInfomation->send_theta,SensorMode::None);//imu:SensorMode::RollPitch
    }
    else
    {
        ros_com->sendContinuousValue(SprintInfo->SpintInfomation->send_x, SprintInfo->SpintInfomation->send_y, 0, SprintInfo->SpintInfomation->send_theta,SensorMode::None);//imu:SensorMode::RollPitch
    }
	//tool->Delay(100); //for stabilit
}
void KidsizeStrategy::detect_aruco_direction(int left_information, int right_information, int left_turn_value, int right_turn_value)
{
    if(SprintInfo->SpintInfomation->SprForWard)
    {
        if (abs(aruco_direction) > right_information)
        {
            SprintInfo->SpintInfomation->tmp_theta += left_turn_value;					//左轉
            ROS_INFO("forward left turn!");
        }
        else if (abs(aruco_direction) < left_information)
        {
            SprintInfo->SpintInfomation->tmp_theta += right_turn_value;					//右轉
            ROS_INFO("forward right turn!");
        }
        else ROS_INFO("forward in mid line!");
    }
    else
    {
        if (abs(aruco_direction) > right_information)
        {
            SprintInfo->SpintInfomation->tmp_theta += right_turn_value;					//右肩向後
            ROS_INFO("backward right shoulder turn!");
        }
        else if (abs(aruco_direction) < left_information)
        {
            SprintInfo->SpintInfomation->tmp_theta += left_turn_value;					//左肩向後
            ROS_INFO("backward left shoulder turn!");
        }
        else ROS_INFO("backward in mid line!");
    }
}
void KidsizeStrategy::aruco_do_forward(void)
{
    ROS_INFO("~~~~~~aruco_do_forward information start~~~~~~");
    if (aruco_distence > ARUCODISTENCE_50)
    {
        ROS_INFO("<50cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_50 - 5, ARUCO_MIDLE_LINE_50 + 5, forward_theta_left_one, forward_theta_right_one);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((aruco_distence <= ARUCODISTENCE_50) && (aruco_distence > ARUCODISTENCE_100))
    {
        ROS_INFO("50cm~100cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_100 - 5, ARUCO_MIDLE_LINE_100 + 5, forward_theta_left_one, forward_theta_right_one);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((aruco_distence <= ARUCODISTENCE_100) && (aruco_distence > ARUCODISTENCE_150))
    {
        ROS_INFO("100cm~150cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_150 - 5, ARUCO_MIDLE_LINE_150 + 5, forward_theta_left_two, forward_theta_right_two);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((aruco_distence <= ARUCODISTENCE_150) && (aruco_distence > ARUCODISTENCE_200))
    {
        ROS_INFO("150cm~200cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_200 - 5, ARUCO_MIDLE_LINE_200 + 5, forward_theta_left_two, forward_theta_right_two);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((aruco_distence <= ARUCODISTENCE_200) && (aruco_distence > ARUCODISTENCE_250))
    {
        ROS_INFO("200caruco_do_forwardm~250cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_250 - 5, ARUCO_MIDLE_LINE_250 + 5, forward_theta_left_three, forward_theta_right_three);
        SprintInfo->SpintInfomation->send_x = min(forward_x_max, SprintInfo->SpintInfomation->send_x + forward_x_add);
    }
    else if ((aruco_distence <= ARUCODISTENCE_250) && (aruco_distence > ARUCODISTENCE_300))
    {	
        ROS_INFO("250cm~300cm");
        detect_aruco_direction( ARUCO_MIDLE_LINE_300 - 5, ARUCO_MIDLE_LINE_300 + 5, forward_theta_left_three, forward_theta_right_three);
        SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_250);
	p=2;
    }
    else if (total_size <= ARUCODISTENCE_300)
    {
		ROS_INFO("300cm~");
	    detect_aruco_direction( ARUCO_MIDLE_LINE_300 - 5, ARUCO_MIDLE_LINE_300 + 5, forward_theta_left_three, forward_theta_right_three);
	    SprintInfo->SpintInfomation->send_x = max(slowdown_initial_x, SprintInfo->SpintInfomation->send_x - slowdown_300);
	    p=2;

    }
    ROS_INFO("~~~~~~aruco_do_forward information end~~~~~~");
}
void KidsizeStrategy::aruco_do_backward(void)
{
    ROS_INFO("~~~~~~~~just backward aruco start~~~~~~~~~");
    if (aruco_distence > ARUCODISTENCE_50)
    {
        ROS_INFO("aruco_back_before_50");
		detect_aruco_direction( ARUCO_MIDLE_LINE_50 - 10, ARUCO_MIDLE_LINE_50 + 10, backward_theta_left_one, backward_theta_right_one);
    }
    else if ((aruco_distence <= ARUCODISTENCE_50) && (aruco_distence > ARUCODISTENCE_100))
    {
        ROS_INFO("aruco_back_between_50~100");
        detect_aruco_direction( ARUCO_MIDLE_LINE_100 - 10, ARUCO_MIDLE_LINE_100 + 10, backward_theta_left_one, backward_theta_right_one);
    }
    else if ((aruco_distence <= ARUCODISTENCE_100) && (aruco_distence > ARUCODISTENCE_150))
    {
        ROS_INFO("aruco_back_between_100~150");
        detect_aruco_direction( ARUCO_MIDLE_LINE_150 - 10, ARUCO_MIDLE_LINE_150 + 10, backward_theta_left_two, backward_theta_right_two);
    }
    else if ((aruco_distence <= ARUCODISTENCE_150) && (aruco_distence > ARUCODISTENCE_200))
    {
        ROS_INFO("aruco_back_between_150~200");
        detect_aruco_direction( ARUCO_MIDLE_LINE_200 - 5, ARUCO_MIDLE_LINE_200 + 5, backward_theta_left_two, backward_theta_right_two);
    }
    else if ((aruco_distence <= ARUCODISTENCE_200) && (aruco_distence > ARUCODISTENCE_250))
    {
        ROS_INFO("aruco_back_between_200~250");
        detect_aruco_direction( ARUCO_MIDLE_LINE_250 - 5, ARUCO_MIDLE_LINE_250 + 5, backward_theta_left_three, backward_theta_right_three);
    }
    else if ((aruco_distence <= ARUCODISTENCE_250) && (aruco_distence > ARUCODISTENCE_300))
    {
        ROS_INFO("aruco_back_between_250~300");
        detect_aruco_direction( ARUCO_MIDLE_LINE_300 - 5, ARUCO_MIDLE_LINE_300 + 5, backward_theta_left_three, backward_theta_right_three);
    }
    else if (total_size <= ARUCODISTENCE_300)
    {
        ROS_INFO("aruco_back_between_300~");
        detect_aruco_direction( ARUCO_MIDLE_LINE_300 - 5, ARUCO_MIDLE_LINE_300 + 5, backward_theta_left_three, backward_theta_right_three);
    }
	SprintInfo->SpintInfomation->send_x = max(backward_x_max, SprintInfo->SpintInfomation->send_x + backward_x_add);	//backward speed limit
    ROS_INFO("~~~~~~~~just backward aruco end~~~~~~~~~");
}
