/**
*   @file       TrackVO.cpp
*
*   @author     Juan Antonio Sanchez Diaz <jsanchez22@us.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of camera odometry using LK Tracking in the viaduct scenarios
**/

#include "track_vo.h"

TrackVO::TrackVO(ros::NodeHandle &_node, std::string node_name)
{
  this->node_ = _node; // ros node handle
  this->name = node_name;

  // Get parameters
  getParameters();

  //Publishers
  ORBi = node_.advertise<sensor_msgs::Image>(imageORB_topic, 100);  
  path = node_.advertise<nav_msgs::Path>(path_topic, 100); 
  odom = node_.advertise<geometry_msgs::PoseStamped>(odom_topic, 100); 
  homop = node_.advertise<nav_msgs::Odometry>(pose_topic, 100); 

  //Subscribers
  image = node_.subscribe(image_topic, 100, &TrackVO::imageCallback, this);
  pose = node_.subscribe(altimeter_topic, 100, &TrackVO::poseCallback, this); 

  srv_manager = node_.advertiseService(mission_manager_service, &TrackVO::node_init, this);
  srv_alarm = node_.serviceClient<mission_manager::Alarms>(alarm_service);

  msg_path.header.frame_id = path_frame_id; //
}

/**
* @brief Destructor of the TrackVO class
**/
TrackVO::~TrackVO()
{
  ROS_INFO("[track_vo]: Node has been killed");
}

void TrackVO::getParameters()
{
  std::string _node_name = this->name;

  node_.getParam(_node_name + "/MIN_POINTS", min_points);  //
  node_.getParam(_node_name + "/NEW_POINTS", new_points);	 // 
  node_.getParam(_node_name + "/INITIAL_POINTS", init_points);  //  
  node_.getParam(_node_name + "/LOST_POINTS", lost_points);  //
  node_.getParam(_node_name + "/EDGE_THRESHOLD", edge_threshold);  //
  node_.getParam(_node_name + "/NROWS", nrows);  //
  node_.getParam(_node_name + "/NCOLS", ncols);  // 
  node_.getParam(_node_name + "/ADVISE_THRESHOLD", advise_threshold);  //
  node_.getParam(_node_name + "/MININUM_DISTANCE", minDistance);  //
  node_.getParam(_node_name + "/BLOCK_SIZE", blockSize);
  node_.getParam(_node_name + "/RESPONSE_K", responseK);
  node_.getParam(_node_name + "/SCALE",escala);  // 
  node_.getParam(_node_name + "/HIGH",min_high);  // 
  node_.getParam(_node_name + "/ODOM_FRAME_ID",odom_frame_id);  // 
  node_.getParam(_node_name + "/PATH_FRAME_ID",path_frame_id);  // 
  node_.getParam(_node_name + "/ROT_XY",XY_homo); // 
  node_.getParam(_node_name + "/ROT_XZ",XZ_homo); // 
  node_.getParam(_node_name + "/INTRINSIC_PARAMETERS",int_matrix); // 
  node_.getParam(_node_name + "/DEBUG",debug); // Debug mode
  node_.getParam(_node_name + "/IMAGE_TOPIC",image_topic); 
  node_.getParam(_node_name + "/ALTIMETER_TOPIC",altimeter_topic); 
  node_.getParam(_node_name + "/PATCH_TOPIC",path_topic); 
  node_.getParam(_node_name + "/POSE_TOPIC",pose_topic); 
  node_.getParam(_node_name + "/ODOM_TOPIC",odom_topic); 
  node_.getParam(_node_name + "/ORB_IMAGE_TOPIC",imageORB_topic); 
  node_.getParam(_node_name + "/MISSION_MANAGER_SERVICE",mission_manager_service); 
  node_.getParam(_node_name + "/ALARM_SERVICE",alarm_service); 

  // Matriz de parametros intrínsecos invertida
  ki << int_matrix[0], int_matrix[1], int_matrix[2],
        int_matrix[3], int_matrix[4], int_matrix[5],
        int_matrix[6], int_matrix[7], int_matrix[8];

  // Print parameters
    ROS_INFO("\n Parameters: \n"
             "      min_points = %d \n"
             "      new_points = %d \n"
             "      init_points = %d \n"
             "      lost_points = %d \n"
             "      edge_threshold = %f \n"
             "      nrows = %d \n"
             "      ncols = %d \n"
             "      advise_threshold = %d \n"
             "      minDistance = %f \n"
             "      blockSize = %d \n"
             "      responseK = %f \n"
             "      escala = %f \n"
             "      min_high = %f \n"
             "      odom_frame_id = %s \n"
             "      path_frame_id = %s \n"
             "      XY_homo = %f \n"
             "      XZ_homo = %f \n"
             "      int_matrix = [%f, %f, %f, %f, %f, %f, %f, %f, %f] \n"
             "      debug = %d \n"             
             "      image_topic = %s \n"
             "      altimeter_topic = %s \n"
             "      path_topic = %s \n"
             "      pose_topic = %s \n"
             "      odom_topic = %s \n"
             "      imageORB_topic = %s \n"
             "      mission_manager_service = %s \n"
             "      alarm_service = %s \n",
             min_points, new_points, init_points, lost_points, edge_threshold,
             nrows, ncols, advise_threshold, minDistance, blockSize, 
             responseK, escala, min_high, odom_frame_id.c_str(), 
             path_frame_id.c_str(),XY_homo, XZ_homo, 
             int_matrix[0], int_matrix[1], int_matrix[2],
             int_matrix[3], int_matrix[4], int_matrix[5],
             int_matrix[6], int_matrix[7], int_matrix[8],
             debug, image_topic.c_str(), altimeter_topic.c_str(),
             path_topic.c_str(), pose_topic.c_str(), odom_topic.c_str(), imageORB_topic.c_str(),
             mission_manager_service.c_str(), alarm_service.c_str());
}

void TrackVO::featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)
{ 
  vector<float> err;					
  TermCriteria termcrit=TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, Size(15,15), 2, termcrit, 0, 0.001);

  int indexCorrection = 0;
  for(int i=0; i<status.size(); i++)
  {  
    Point2f pt = points2.at(i - indexCorrection);
    if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	
    {
      if((pt.x<0)||(pt.y<0)) status.at(i) = 0;
      points1.erase(points1.begin() + (i - indexCorrection));
      points2.erase(points2.begin() + (i - indexCorrection));
      indexCorrection++;
    }
  }
}

void TrackVO::drawFeatures(Mat frame, vector<Point2f>& points)
{
  for( size_t i = 0; i < points.size(); i++ ) 
  {
    circle(frame, points[i], 1, Scalar(0, 0, 255), 10);
  }
}

void TrackVO::featureDetection(Mat img_1, vector<Point2f>& points, int maxCorners)
{   
  double qualityLevel = 0.4;
  double minDistance = 30;
  int blockSize = 7;
  double k = 0.04;
  goodFeaturesToTrack(img_1, points, maxCorners, qualityLevel, minDistance, Mat(), blockSize, false, k);
}

tuple<int, int> TrackVO::minFeatureBox(Mat img, vector<Point2f>& points, int ncols, int nrows, int threshold)
{
    std::tuple<int, int> myTuple; // empty tuple
    int minCont = 10000;
    float width = img.cols, height = img.rows;
    for(int ni = 0; ni < ncols; ni++)
    {
        for(int nj = 0; nj < nrows; nj++)
        {
            int cont = 0;
            for(int k = 0; k < points.size(); k++)
            {
                //printf("Punto[%d]:", k); 
                //cout << points[k] << endl; 
                if ((points[k].y > (nj*height/nrows)) && (points[k].y < (nj+1)*height/nrows) && (points[k].x > (ni*width/ncols)) && (points[k].x < (ni+1)*width/ncols))
                {
                    cont++;
                }
            }

            if (cont < minCont){
                // cout << "Puntos en la celda:" << cont << endl; 
                myTuple = make_tuple(ni, nj);
                minCont = cont;
            } 
        }
    }

    if (minCont < threshold) // si el cont de box minimo es inferior al limite, devuelve box
    {
        return myTuple;
    }
    else
    {
        return make_tuple(-1, -1);
    }
}

void TrackVO::equalize_ycrcb(Mat img)
{
    if(img.channels() >= 3)
    {
        Mat ycrcb;

        cvtColor(img,ycrcb,COLOR_BGR2YCrCb);

        vector<Mat> channels;
        split(ycrcb,channels);

        equalizeHist(channels[0], channels[0]);

        Mat result;
        merge(channels,ycrcb);

        cvtColor(ycrcb,img,COLOR_YCrCb2BGR);
    }
}

void TrackVO::poseCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    if(!takeoff)
    {
        geometry_msgs::PoseStamped m;

        z = msg->range; //msg->range

        m.pose.position.x = (XG + xa) / 2; //* escala;
        m.pose.position.y = (YG + ya) / 2; //* escala;
        m.pose.position.z = z;

        // Obtain a quaternion from the estimated yaw
        tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, YAWG); //- XY_homo*3.1416/180
        geometry_msgs::Quaternion _q_pose;
        _q_pose.x = q_t.x();
        _q_pose.y = q_t.y();
        _q_pose.z = q_t.z();
        _q_pose.w = q_t.w();
        m.pose.orientation = _q_pose;

        if(msg->range == 0.0) // TODO: MANDAR SERVICIO DE ALARMA
        {
            // cout << "ERROR EN LA LECTURA DE ALTÍMETRO" << endl;
            mission_manager::Alarms srv;
            srv.request.type = mission_manager::Alarms::Request::Obstacle;

            srv_alarm.call(srv);

            Alarm_FLAG = true;
            Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;

            m.pose.position.x = xa;
            m.pose.position.y = ya;
            m.pose.position.z = za;
        }

        if(debug)  std::cout << "Z " << z - za << std::endl;
        iz = z - za;

        xa = m.pose.position.x;
        ya = m.pose.position.y;
        za = m.pose.position.z;

        float xrot = 0;
        float yrot = 0;
        float zrot = 0; 

        xrot = m.pose.position.x;
        yrot = m.pose.position.y;
        zrot = m.pose.position.z;

        // XY Rotation
        float xtrans = xrot;
        float ytrans = yrot;
        float theta = XY_homo * 3.1416/180; 
        xrot = xtrans*cos(theta) + ytrans*sin(theta);
        yrot = -xtrans*sin(theta) + ytrans*cos(theta); 

        // XZ Rotation
        xtrans = xrot;
        float ztrans = zrot;
        theta = XZ_homo * 3.1416/180;
        xrot = xtrans*cos(theta) + ztrans*sin(theta);
        zrot = -xtrans*sin(theta) + ztrans*cos(theta);

        m.pose.position.x = xrot;
        m.pose.position.y = yrot;
        m.pose.position.z = zrot;

        msg_path.poses.push_back(m);
        path.publish(msg_path);
        msg_path.header.seq++;

        nav_msgs::Odometry pose_msg;
        pose_msg.pose.pose = m.pose;
        pose_msg.header.frame_id = odom_frame_id;
        homop.publish(pose_msg);
    }
}

// Función para calcular la distancia entre dos puntos 2D
float calcularDistancia(const Point2f& p1, const Point2f& p2) {
    return norm(p1 - p2);
}

// Función para eliminar puntos cercanos en la nube
std::vector<Point2f> eliminarPuntosCercanos(std::vector<Point2f> nubeDePuntos, float distanciaMaxima) {
    std::vector<Point2f> nubeFiltrada;

    for (int i = 0; i < nubeDePuntos.size(); i++) {
        bool mantener = true;

        for (int j = 0; j < nubeFiltrada.size(); j++) {
            float distancia = calcularDistancia(nubeDePuntos[i], nubeFiltrada[j]);
            if (distancia <= distanciaMaxima) {
                mantener = false;
                break;
            }
        }

        if (mantener) {
            nubeFiltrada.push_back(nubeDePuntos[i]);
        }
    }

    return nubeFiltrada;
}

void TrackVO::imageCallback(const sensor_msgs::Image::ConstPtr& image){
    HealthCount = 0;
    if(Alarm_FLAG)
    {
        mission_manager::Alarms srv;
        srv.request.type = mission_manager::Alarms::Request::SensorsHealth; // TODO: CAMBIAR EL TIPO DE ALARMA
        srv.request.state = false; // ¿?
        srv.request.msg = "Monocular camera working";
        srv_alarm.call(srv);

        Alarm_FLAG = false;
        Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;
    }

    if(!takeoff){
        ros::WallTime start_, end_;
        start_ = ros::WallTime::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); 

        int x = 0; // Coordenada x de inicio
        int y = 0; // Coordenada y de inicio
        int width = 853; // 1280 * 2/3
        int height = 720; 
        cv::Rect roi(x, y, width, height);

        if(init){
            old_img = cv_ptr->image;
            old_img = old_img(roi);

            equalize_ycrcb(old_img);
            cvtColor(old_img, old_gray, COLOR_BGR2GRAY); 
            width = old_gray.cols;
            height = old_gray.rows;

            int ncols = 2, nrows = 1;
            // boxFeature(old_gray, points1, ncols, nrows);
            featureDetection(old_gray, points1, 100);

            drawFeatures(old_img, points1);

            init = false;
        }else{
            frame_img = cv_ptr->image;
            frame_img = frame_img(roi);

            equalize_ycrcb(frame_img);
            cvtColor(frame_img, frame_gray, COLOR_BGR2GRAY);

            if (!old_gray.empty() && !frame_gray.empty()){
                featureTracking(old_gray, frame_gray, points1, points2, status); 
                Mat A(2*points1.size(), 4, CV_64FC1);
                Mat b(2*points1.size(), 1, CV_64FC1);
                for(int i = 0; i < points1.size(); i++){
                    A.at<double>(i*2,0) = points1[i].x - width / 2; A.at<double>(i*2,1) = - (points1[i].y - height / 2); A.at<double>(i*2,2) = 1; A.at<double>(i*2,3) = 0; 
                    A.at<double>(i*2 + 1,0) = points1[i].y - height / 2; A.at<double>(i*2 + 1,1) = points1[i].x - width / 2; A.at<double>(i*2 + 1,2) = 0; A.at<double>(i*2 + 1,3) = 1; 
                    b.at<double>(i*2,0) = points2[i].x - width / 2;
                    b.at<double>(i*2 + 1,0) = points2[i].y - height / 2;
                }
                
                Eigen::MatrixXd Ae;
                Eigen::MatrixXd be;
                cv2eigen(A,Ae);
                cv2eigen(b,be);
                Eigen::VectorXd xopt = Ae.colPivHouseholderQr().solve(be);
                float u = xopt(2);
                float v = xopt(3);

                // cout << "theta: " << atan(xopt(1)/xopt(0))*180/CV_PI << endl;
                // cout << "Px: " << u << endl;
                // cout << "Py: " << v << endl;

                Eigen::VectorXd incI(3);
    
                incI << (ki(0,0)*u + ki(0,1)*v), //+ ki(0,2)
                        (ki(1,0)*u + ki(1,1)*v), //+ ki(1,2)
                        (ki(2,0)*u + ki(2,1)*v); //+ ki(2,2)

                float xi = incI[0];
                float yi = incI[1];
            
                if(xi < 0.06 && yi < 0.06){
                    geometry_msgs::PoseStamped m_odom;
                    tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, atan(xopt(1)/xopt(0))); //- XY_homo*3.1416/180
                    geometry_msgs::Quaternion _q_pose;
                    _q_pose.x = q_t.x();
                    _q_pose.y = q_t.y();
                    _q_pose.z = q_t.z();
                    _q_pose.w = q_t.w();
                    m_odom.pose.orientation = _q_pose;
                    m_odom.pose.position.x = - xi * z * escala;
                    m_odom.pose.position.y = - yi * z * escala;
                    m_odom.pose.position.z = iz;
                    odom.publish(m_odom);

                    YAWG += atan(xopt(1)/xopt(0));
                    XG = XG - (xi * cos(YAWG) + yi * sin(YAWG)) * z;
                    YG = YG + (-xi * sin(YAWG) + yi * cos(YAWG)) * z;
                }

                if (abs(int(points2.size()) - dif) >= 0){ // si se pierden mas de 2 puntos comprobamos
                    // Puntos para el fragmento de la imagen
                    vector<Point2f> pointsf;
                    int nrows = 3, ncols = 4;
                    int ni, nj;
                    std::tie(ni, nj) = minFeatureBox(frame_gray, points2, ncols, nrows, 7);
                    
                    if(debug) cout << "nrow: " << ni << " ncol: " << nj << endl;
                    
                    if(ni != -1 && nj != -1){ // Recalculo puntos si se han perdido
                        vector<Point2f> pointsf;
                        Mat box(height/nrows,width/ncols, CV_8UC1);

                        for(int i = 0; i < width/ncols; i++){    
                            for(int j = 0; j < height/nrows; j++){
                                box.at<uint8_t>(j, i) = frame_gray.at<uint8_t>(j+nj*height/nrows, i+ni*width/ncols); 
                            }
                        }

                        featureDetection(box, pointsf, 15);
                        //cout << pointsf.size() << endl;

                        for(int k = 0; k < pointsf.size(); k++){
                            pointsf[k].x = pointsf[k].x + ni*width/ncols;
                            pointsf[k].y = pointsf[k].y + nj*height/nrows;
                        }
                        points2.insert(points2.end(), pointsf.begin(), pointsf.end()); // Añado los nuevos puntos

                        dif = points2.size(); // Reiniciamos contador y puntos perdidos
                    }
                }

                end_ = ros::WallTime::now();
                double execution_time = (end_ - start_).toNSec() * 1e-6;
                // cout << execution_time << endl;    

                drawFeatures(cv_ptr->image, points2);
                ORBi.publish(cv_ptr->toImageMsg());

                old_gray = frame_gray.clone();
                old_img = frame_img.clone();

                if(int(points2.size()) < 20){
                    featureDetection(frame_gray, points1, 100);
                } 
                else if(int(points2.size()) > 300){
                    points1 = eliminarPuntosCercanos(points2, 40);
                }
                else{
                    points1 = points2;
                }
                
                if(debug)
                {
                    cout << "xi: " << xi << endl;
                    cout << "yi: " << yi << endl;
                    cout << "YAWG: " << YAWG << endl;
                    cout << "XG: " << XG << endl;
                    cout << "YG: " << YG << endl; 
                    cout << "Puntos actuales: " << int(points2.size()) << endl;
                    cout << "Puntos perdidos: " << abs(int(points2.size()) - dif) << endl; // " n: " << n << 
                    cout << "###########################################################" << endl;
                }
                points2.clear();       
            }
        }
    }
}

bool TrackVO::node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res){
    cout << "[CAMERA]: service received" << endl;
    XG = req.x_pos;
    YG = req.y_pos;
    //YAWG = 0.0;
    YAWG = req.yaw_rot;
    //YAWG = -XY_homo*3.1416/180;
    xa = XG;
    ya = YG;

    takeoff = req.takeoff;
    return true;
}

/**
      * @brief ROS spin loop
*/
void TrackVO::spin(){
    ros::spin();
}

void TrackVO::spinOnce(){
    ros::spinOnce();
    HealthCount++; // Wrapping
    if(HealthCount > 15 && !Alarm_FLAG){
        mission_manager::Alarms srv;
        srv.request.type = mission_manager::Alarms::Request::SensorsHealth; // TODO: CAMBIAR EL TIPO DE ALARMA
        srv.request.state = true; // ¿?
        srv.request.msg = "Monocular camera not working";
        srv_alarm.call(srv);

        Alarm_FLAG = true;
        Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;
    }
}