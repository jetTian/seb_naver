#include "mpc/mpc.h"
 
using namespace std;
 
void MPC::init(ros::NodeHandle &nh)
{
    vector<double> Q_ = {10, 10, 0.5};
    vector<double> R_ = {10, 10, 0.5};
    vector<double> Rd_ = {10, 10, 0.5};
    nh.param("mpc/dt", dt, -1.0);
    nh.param("mpc/predict_steps", Npre, -1);
    nh.param("mpc/max_steer", max_steer, -1.0);
    nh.param("mpc/max_dsteer", max_dsteer, -1.0);
    nh.param("mpc/max_speed", max_speed, -1.0);
    nh.param("mpc/min_speed", min_speed, -1.0);
    nh.param("mpc/max_accel", max_accel, -1.0);
    nh.param("mpc/wheel_base", wheel_base, -1.0);
    nh.param("mpc/delay_num", delay_num, -1);
    nh.param("mpc/test_mpc", test_mpc, false);
    nh.param<std::vector<double>>("mpc/matrix_q", Q_, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_r", R_, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_rd", Rd_, std::vector<double>());

    Q = diag(DM({Q_[0], Q_[1], Q_[2]}));
    R = diag(DM({R_[0], R_[1]}));
    Rd = diag(DM({Rd_[0], Rd_[1]}));
    U_min = DM({min_speed, -max_steer});
    U_max = DM({max_speed, max_steer});
    x_0.resize(3, 1);
    x_0 = {0, 0, 0};

    u_0.resize(2, 1);
    u_0 = {0, 0};
    X_sol = repmat(x_0, 1, Npre);  //求解器的状态X 初始设置为0，repmat(A, M, N) 大小为size(A,1)*M, size(A,2)*N，这里为3行,1*Npre列
    U_sol = repmat(u_0, 1, Npre);  //求解器的控制U 初始设置为0，这里为2行,1*Npre列

    nh.param("mpc/bk_mode", bk_mode, false);
    nh.param<string>("mpc/traj_file", traj_file, "xxx");

    has_odom = false;
    receive_traj = false;
    for (int i=0; i<delay_num; i++)
        output_buff.push_back(Eigen::Vector2d::Zero());
    cmd.velocity = 0.0;
    cmd.angle_velocity = 0.0;

    pos_cmd_pub_ = nh.advertise<tank_sdk::TankCMD>("cmd", 200);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/following_path", 10);
    predict_pub = nh.advertise<visualization_msgs::Marker>("/predict_path", 10);
    ref_pub = nh.advertise<visualization_msgs::Marker>("/reference_path", 10);
    desiredS_pub_ = nh.advertise<mpc_controller::desiredState>("/desired_state",10);
    cmd_timer_ = nh.createTimer(ros::Duration(0.05), &MPC::cmdCallback, this);
    odom_sub_ = nh.subscribe("odom", 1, &MPC::rcvOdomCallBack, this);
    traj_sub_1 = nh.subscribe("se2traj", 1, &MPC::rcvTrajCallBack, this);
    traj_sub_2 = nh.subscribe("traj", 1, &MPC::rcvFlatTrajCallBack, this);
    traj_sub_3 = nh.subscribe("dptraj", 1, &MPC::rcvDPTrajCallBack, this);
    err_pub = nh.advertise<std_msgs::Float64>("/track_err", 10);
    if (test_mpc)
    {
        trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &MPC::rcvTriggerCallBack, this);
    }

    errs.clear();
    vels.clear();
    angle_vels.clear();
    jerks.clear();
    errs_yaw.clear();
    errs_lon.clear();
    errs_lat.clear();

    trajectory_length = 0.0;

    if (bk_mode)
    {
        string o = traj_file;
        outfile.open(o.insert(o.find("trajs"), "err_"), std::ofstream::out);
        outfile.clear();
    }
}

void MPC::rcvTriggerCallBack(const geometry_msgs::PoseStamped msg)
{
    receive_traj = true;
    traj_analyzer.setTestTraj(max_speed*0.5);
    eight_path = traj_analyzer.getTajWps(0.1);
}

void MPC::rcvTrajCallBack(mpc_controller::SE2TrajConstPtr msg)
{
    traj_analyzer.setTraj(msg);
    receive_traj = true;
}

void MPC::rcvFlatTrajCallBack(mpc_controller::PolyTrajConstPtr msg)
{
    traj_analyzer.setTraj(msg);
    receive_traj = true;
    has_output = false;
}
void MPC::rcvDPTrajCallBack(mpc_controller::DPtrajContainerConstPtr msg){
    new_traj_analyzer.setTraj(msg);
    receive_traj = true;
    has_output = false;
}
void MPC::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
    has_odom = true;
    trajectory_length += Eigen::Vector2d(now_state.x-msg->pose.pose.position.x, now_state.y-msg->pose.pose.position.y).norm();
    now_state.x = msg->pose.pose.position.x;
    now_state.y = msg->pose.pose.position.y;
    now_state.theta = 2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

/*
  定时回调
  获取当前状态和参考轨迹
  执行控制器逻辑(调用getCmd()进行MPC求解)
  发布控制指令和可视化信息
  统计和输出控制误差
*/
void MPC::cmdCallback(const ros::TimerEvent &e)
{
    drawFollowPath();  //可视化当前路径

    if (!has_odom || !receive_traj){  //如果没有接收到里程计或轨迹，直接返回，不做任何控制操作
        return;
    }
    ros::Time begin = ros::Time::now();
    if(begin > new_traj_analyzer.start_time){  //判断是否要用新的轨迹分析器
        traj_analyzer = new_traj_analyzer;
    }
    xref = traj_analyzer.getRefPoints(Npre, dt);  //获取参考轨迹点
    if (!xref.empty())
    {
        TrajPoint p_now = traj_analyzer.getRefPoint();
        Eigen::Vector2d err_vec(p_now.x - now_state.x, p_now.y - now_state.y);  //计算当前参考点与实际位置之间的误差
        std_msgs::Float64 err_msg;
        if(err_vec.norm()<1.0&&err_vec.norm()>0.001){
            errs.push_back(err_vec.norm());
            err_msg.data = errs.back();
            err_pub.publish(err_msg);  //发布误差信息（用于监控或调试）
        }
        {  //计算当前位置与参考轨迹点之间的跟踪误差与偏航角误差，并将其封装到消息中发布出去，以便用于监控、调试或上层控制模块使用
            TrajPoint tp = traj_analyzer.getRefPoint();
            mpc_controller::desiredState ds;
            ds.px = tp.x;
            ds.py = tp.y;
            ds.yaw = tp.theta;
            ds.stamp = ros::Time::now();
            Eigen::Vector2d err_vec(tp.x - now_state.x, tp.y - now_state.y);  //计算当前位置与期望位置的平面距离即L2的误差
            ds.tkerror = err_vec.norm();
            double err_yaw = tp.theta - now_state.theta;  // 航向角的误差
            if(ds.tkerror > 1.0){  //如果误差太大，说明离目标太远，忽略误差。可能是启动初期/漂移严重的保护
                ds.tkerror = 0.0;
                err_yaw = 0.0;
            }
            else{
                while (err_yaw > M_PI)  //标准化到(-pi,pi)防止+-2pi的跳变
                    err_yaw -= 2.0 * M_PI;
                while (err_yaw < -M_PI)
                    err_yaw += 2.0 * M_PI;
                Eigen::Vector2d dir,n;
                dir << cos(tp.theta), sin(tp.theta);  //表示参考点朝向的单位向量
                n << -sin(tp.theta), cos(tp.theta);
            }
            ds.tkyaw = err_yaw;
            errs_yaw.push_back(ds.tkyaw);
            desiredS_pub_.publish(ds); //包含了期望状态以及L2距离误差+yaw误差
        }
    }
    //publish desired state
    
    /*
    如果到达目标点
    设置速度和转角速度为0
    打印或保存误差统计信息（最大误差、平均误差等）
    重置状态变量，如误差容器等
    */
    if (traj_analyzer.at_goal)
    {
        cmd.velocity = 0.0;
        cmd.angle_velocity = 0.0;

        if (bk_mode)  //bk_mode 默认为false 用于在outfile中记录误差数据
        {
            Eigen::Vector3d initp = Eigen::Vector3d::Zero();
            double mean_err = std::accumulate(errs.begin(), errs.end(), 0.0) / (1.0 * errs.size());
            mean_err_all += mean_err;
            outfile << mean_err << std::endl;
            if (initp == Eigen::Vector3d::Zero())
            {
                outfile << "all_mean_track_err: " << mean_err_all <<std::endl;
                outfile.close();
                ROS_WARN("all_mean_track_err: %lf", mean_err_all);
                ROS_WARN("Benchmark Done.");
                receive_traj = false;
                return;
            }
            geometry_msgs::Point init_point;

            init_point.x = initp.x();
            init_point.y = initp.y();
            init_point.z = initp.z();

            receive_traj = false;
            errs.clear();
            errs_yaw.clear();
            ros::Duration(1.0).sleep();
            eight_path = traj_analyzer.getTajWps(0.1);
            receive_traj = true;
        }

        double mean_err = std::accumulate(errs.begin(), errs.end(), 0.0) / (1.0 * errs.size());
        for (int i=0; i<vels.size(); i++)
            vels[i] = fabs(vels[i]);

        if (!has_output)
        {
            std::cout<<"\033[31m <<<Statistical Datas>>>:"<<std::endl;
            std::cout <<"max tracking-error: "<<*max_element(errs.begin(),errs.end())<<std::endl;
            std::cout <<"max tkyaw-error: "<<*max_element(errs_yaw.begin(),errs_yaw.end())<<std::endl;
            std::cout<<"mean tracking-error: "<<mean_err<<std::endl;
            std::cout<<"trajectory length  : "<<trajectory_length<<std::endl;
            std::cout<<"end position err   : "<<errs.back()<<std::endl;
            

            TrajPoint p_now = traj_analyzer.getRefPoint();
            double yaw_err = p_now.theta - now_state.theta;
            while (yaw_err > M_PI)
                yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI)
                yaw_err += 2.0 * M_PI;
            std::cout<<"end yaw err        : "<<fabs(yaw_err)<<"\033[0m"<<std::endl;

            errs.clear();
            vels.clear();
            angle_vels.clear();
            trajectory_length = 0.0;
            has_output = true;
        }
    }
    else  //未到达目标，执行控制计算
    {
        smooth_yaw(xref);
        ros::Time t1 = ros::Time::now();
        getCmd();  //进行优化计算，得到控制指令
        ros::Time t2 = ros::Time::now();
        vels.push_back(cmd.velocity);
        angle_vels.push_back(fabs(cmd.angle_velocity));
    }
    pos_cmd_pub_.publish(cmd);  //发布控制指令
}

void MPC::getCmd(void)
{
    nlp = casadi::Opti();  //定义优化器对象，即CasADi优化问题的接口为Opti
    casadi::Dict options;  //设置优化器参数，如关闭打印信息、设置为稀疏矩阵等
    casadi::Dict qp_options;  //设置优化器参数
    options["print_status"] = false;
    options["print_time"] = false;
    options["print_header"] = false;
    options["print_iteration"] = false;
    options["verbose"] = false;
    options["verbose_init"] = false;
    qp_options["printLevel"] = "none";
    qp_options["sparse"] = true;
    qp_options["error_on_fail"] = false;
    options["qpsol_options"] = qp_options;

    //设置决策变量
    X = nlp.variable(3, Npre);  //状态: x y theta  
    U = nlp.variable(2, Npre);  //控制 v + steer angle
    J = 0;                      //代价函数初始化
    MX X_0 = nlp.parameter(3, 1);  //初始状态参数
    MX x_next; 
    Slice all;  //？
    
    for (int i = 0; i < Npre; i++)  //构建代价函数与约束，Npre为预测步长
    {
        MX ref_x = xref[i].x;
        MX ref_y = xref[i].y;
        MX ref_theta = xref[i].theta;
        MX ref_state = vertcat(ref_x, ref_y, ref_theta);
        MX ref_v = 0;
        MX ref_steer = 0;
        MX ref_u = vertcat(ref_v, ref_steer);

        //对于每一个预测步
        //状态转移约束,保证状态满足动态模型
        if (i==0)
            x_next = stateTrans(X_0, U(all, i));
        else
            x_next = stateTrans(X(all, i-1), U(all, i));
        nlp.subject_to(X(all, i) == x_next);
        //输入约束
        nlp.subject_to(U_min <= U(all, i) <= U_max);
        
        MX delta_x = ref_state - X(all, i);
        MX delta_u = ref_u - U(all, i); //实际上就是u, 因为ref_u都是0
        
        MX du;
        if (i > 0)
            du = U(all, i) - U(all, i - 1);
        else
            du = U(all, i) - u_0;
        //目标函数 = Q * delta_x*T（跟踪误差） + R * delta_u_*T （控制量大小） + Rd * du*T（控制量变化，平滑性）
        J = J + mtimes(delta_x.T(), mtimes(Q, delta_x));  //？mtimes是什么？
        J = J + mtimes(delta_u.T(), mtimes(R, delta_u));
        J = J + mtimes(du.T(), mtimes(Rd, du));
    }
    x_0 = {now_state.x, now_state.y, now_state.theta};  //设置初始状态与初值
    nlp.set_value(X_0, x_0);
    nlp.set_initial(X, X_sol);
    nlp.set_initial(U, U_sol);
 
    nlp.solver("sqpmethod", options);  //设置求解器
    nlp.minimize(J);
 
    try
    {
        const casadi::OptiSol sol = nlp.solve();  //求解

        X_sol = sol.value(X);  //提取最优解的预测状态
        U_sol = sol.value(U);  //提取最优解的控制
        DM cmd_0 = U_sol(all, 0);  //提取当前最近的一次控制量
        u_0 = cmd_0;
        cmd.velocity = (double)cmd_0(0, 0);  //控制指令 速度
        cmd.angle_velocity = (double)cmd_0(1, 0);  //控制指令 角速度(rad/s), 但实际上是steer角度，前轮转角 steering angle（rad）
    }
    catch(const std::exception& e)
    {
        // ROS_INFO_STREAM("Initial or symbolic values of X:\n" << nlp.debug().value(X));
        // ROS_INFO_STREAM("Initial or symbolic values of U:\n" << nlp.debug().value(U));
        // ROS_WARN_STREAM("Failed with initial X: " << X_sol);
        // ROS_WARN_STREAM("Failed with initial U: " << U_sol);
        // ROS_WARN("solver error, but we ignore.");
    }
    
    drawRefPath();  //可视化参考轨迹
    drawPredictPath();  //可视化预测轨迹

    if (delay_num>0)  //更新控制缓存
    {
        output_buff.erase(output_buff.begin());
        output_buff.push_back(Eigen::Vector2d(cmd.velocity, cmd.angle_velocity));
    }
}
