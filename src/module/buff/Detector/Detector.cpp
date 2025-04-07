#include "../../../../include/module/buff/Detector/Detector.hpp"

// 图像处理
cv::Mat Detector::getSpliteImg(const cv::Mat& src,const ColorType& colorType)
{
    cv::Mat cvtGray;
    cv::Mat subGray;

    cv::cvtColor(src, cvtGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Mat> splited;
    cv::split(src, splited);

    if(colorType == Red)
    {
        // 红
        threshold(cvtGray, cvtGray, 150, 255, cv::THRESH_BINARY);//150
        subtract(splited[2], splited[0], subGray);
        threshold(subGray, subGray, 112, 255, cv::THRESH_BINARY);//200

    }
    else
    {
        // 蓝
        threshold(cvtGray, cvtGray, 170, 255, cv::THRESH_BINARY);
        subtract(splited[0], splited[2], subGray);
        // 120 130 暂时稳定
        threshold(subGray, subGray, 120, 255, cv::THRESH_BINARY);//110
    }

//    cv::Mat blur;
//    cv::GaussianBlur(subGray,blur,cv::Size(5,5),0);// 3 5 7

    //洞
    cv::Mat result =  fillHole(subGray & cvtGray);
    return subGray;
}


// 未写完，暂时不用
// 找出蓝色区域部分没有父轮廓的最小轮廓，且该轮廓外接矩形的中心点离已知的一个点最近
//bool Detector::findCircle(const cv::Mat&src,const cv::Mat&mask/*,cv::Point2f& fitCircle*/)
//{
//    std::vector<std::vector<cv::Point>>contours;
//    std::vector<cv::Vec4i>hierarchy;
//    cv::findContours(mask,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE,cv::Point2f(0,0));

//  //    cv::Point2d fitcircle;//圆心
//  //    double radius = 0;//半径
//  //    CircleFitting(points,fitcircle,radius);

//    std::vector<std::vector<cv::Point>> top_level_contours;
//    for(size_t i=0;i<contours.size();i++){
//        if(hierarchy[i][3]==-1){ // 无父轮廓
//            top_level_contours.push_back(contours[i]);
//        }
//    }

//    if(!top_level_contours.empty()){
//        // 计算各轮廓面积
//        std::vector<double>areas;
//        for(const auto& cnt:top_level_contours){
//            areas.push_back(contourArea(cnt));
//        }

//        // 找出最小面积
//        double min_area=*min_element(areas.begin(),areas.end());
//        std::vector<cv::Rect>min_rects;
//        for(size_t i=0;i<areas.size();i++){
//            if(areas[i]==min_area){
//                min_rects.push_back(boundingRect(top_level_contours[i]));
//            }
//        }

//        //cv::Point2d center;
//        // 找出离已知点最近的矩形
//  //        double min_dist=DBL_MAX;
//  //        cv::Rect closest_rect;
//  //        for(const auto&rect : min_rects){
//  //            cv::Point2d center(rect.x+rect.width/2,rect.y+rect.height/2);
//  //            double dist=cv::norm(center-fitcircle);
//  //            if(dist<min_dist){
//  //                min_dist=dist;
//  //                closest_rect=rect;
//  //            }
//  //        }

//  //        cv::Point2d center(closest_rect.x+closest_rect.width/2,
//  //                           closest_rect.y+closest_rect.height/2);

//       //   cv::circle(src, center, 5, cv::Scalar(0, 255, 0), -1);

//    }

//    return true;
//}



// 核心函数
bool Detector::findCenterPoints(const cv::Mat&src,const cv::Mat&mask){
    this->Start_time = std::chrono::high_resolution_clock::now();
    // 轮廓分析
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    const auto contour_info = analyzeContours(mask);
    if (validateContour(contour_info)) {
        processValidContour(src, contours, contour_info);
        updateTrackingState();
        drawCrucialInfo(src);
        return true;
    }
    return false;
}


Detector::ContourInfo Detector::analyzeContours(const cv::Mat &mask) {
    ContourInfo info;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);

    const size_t num_contours = contours.size();
    if (num_contours == 0) {
        return info; // 无轮廓时直接返回
    }

    // 计算所有轮廓的面积
    std::vector<double> areas(num_contours);
    for (size_t i = 0; i < num_contours; i++) {
        areas[i] = cv::contourArea(contours[i]);
        if (areas[i] > 3600) {
            info.max_area_index = i;
        }
    }

    // 计算每个轮廓的子轮廓数量
    std::vector<int> child_counts(num_contours, 0);
    for (size_t j = 0; j < num_contours; j++) {
        const int parent = hierarchy[j][3];
        if (parent != -1 && static_cast<size_t>(parent) < num_contours) {
            child_counts[parent]++;
        }
    }

    // 找出子轮廓数量最多的父轮廓
    for (size_t i = 0; i < num_contours; i++) {
        const int current_child_count = child_counts[i];
        if (current_child_count > info.max_child_count) {
            info.max_child_count = current_child_count;
            info.max_child_index = static_cast<int>(i);
        }
    }

    // 计算最终选中的轮廓面积和周长
    if (info.max_child_index != -1) {
        const int idx = info.max_child_index;
        if (idx >= 0 && static_cast<size_t>(idx) < num_contours) {
            const auto& contour = contours[idx];
            if (!contour.empty()) {
                info.lenth_Area = areas[idx];
                info.max_contour_length = cv::arcLength(contour, true);
            }
        }
    }

    return info;
}


// 0 条件筛选、判断
bool Detector::validateContour(const ContourInfo& info) const {
    const bool area_valid = (info.lenth_Area > 3200) && (info.lenth_Area < 6500);
    const bool length_valid = (info.max_contour_length > 250) && (info.max_contour_length < 400);
    const bool activation_valid = (activation_count <= 5);

    return (info.max_area_index!=-1)&&(info.max_child_index!=-1)&&
            area_valid&&length_valid &&activation_valid;
}


// 有效轮廓处理
void Detector::processValidContour(const cv::Mat& src,const std::vector<std::vector<cv::Point>>& contours,
                                  const ContourInfo& info) {
    // 计算旋转矩形
    const auto& contour = contours[info.max_child_index];
    const cv::RotatedRect rect = cv::minAreaRect(contour);
    this->buff_center = rect.center;

    this->currentTime = std::chrono::high_resolution_clock::now();

    // 处理顶点
    cv::Point2f vertices[4];
    circle(src, cv::Point(rect.center.x, rect.center.y), 5, cv::Scalar(0, 255, 0), -1, 8);
    rect.points(vertices);

    //for (int j = 0; j < 4; ++j)
    //{
         //line(src, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2,8);
    //}
     double r1=getDistance(vertices[0],vertices[1])/2.0;
     double r2=getDistance(vertices[1],vertices[2])/2.0;
     double r3=getDistance(vertices[2],vertices[3])/2.0;
     double r4=getDistance(vertices[3],vertices[0])/2.0;
     const double R=(r1+r2+r3+r4)/4;

     // 拷贝
     this->rect_points.assign(vertices,vertices+4);
     // 内切圆
     cv::circle(src,cv::Point(rect.center.x,rect.center.y),R,cv::Scalar(0,255,0),2,8);

    // 计算拟合圆
    double radius = 0;
    CircleFitting(radius);

    // 预测位置
    // 判断旋转方向
    this->is_clockwise = judgeRotation();
    // 预判打击点
    predictPosition();
    if(this->ModeType==0)
    circle(src,large_buff_points_center,8,cv::Scalar(0,0,255),2,8);
    else
    circle(src,small_buff_points_center,8,cv::Scalar(0,0,255),2,8);//圈
    circle(src,fit_center,radius,cv::Scalar(0,255,0),1,8);

}


// 目标变化状态
void Detector::updateTrackingState() {
    this->activation_count += (abs(buff_center.x-prev_point.x)>150||abs(buff_center.y-prev_point.y)>150)?1:0;
    this->prev_point = buff_center;

    this->prevTime = std::chrono::high_resolution_clock::now();
    this->time_diff=abs(std::chrono::duration<double>(currentTime-prevTime).count());

    std::cout<<"time_diff: "<<time_diff<<std::endl;

    this->tracking_points.emplace_back(buff_center);
}


// 预测位置
void Detector::predictPosition() {
    // 统一的位置预测逻辑
    if (/*SpeedState(0.1, 0.1)*/Mode::LARGE) {
        this->ModeType=0; // Large Buff
        std::cout<<"variable speed point"<<std::endl;
        LargeBuffPosition();
        this->large_buff_points_center=getCenter(large_buff_points);
    }
    else {
        this->ModeType=1; // Small Buff
        std::cout<<"Small-uniform speed point"<<std::endl;
        SmallBuffPosition();
        this->small_buff_points_center=getCenter(small_buff_points);
    }
}


// 调试信息绘制
void Detector::drawCrucialInfo(const cv::Mat& src)const{
//    if (activation_count >= 5) {
//        cv::putText(src, "Activated successfully", {200, 42},
//                   cv::FONT_HERSHEY_SIMPLEX, 1.5, {0, 255, 0}, 1);
//    }

    cv::putText(src, "Count : " + std::to_string(this->activation_count),
               {0, 142}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 1);
}



bool Detector::judgeRotation(){

    // 坐标系转换：图像坐标系 -> 数学坐标系
    const float prev_dx = prev_point.x - fit_center.x;
    const float prev_dy = -(prev_point.y - fit_center.y);  // 反转Y轴
    const float prev_angle = std::atan2(prev_dy, prev_dx);

    const float curr_dx = buff_center.x - fit_center.x;
    const float curr_dy = -(buff_center.y - fit_center.y);
    const float curr_angle = std::atan2(curr_dy, curr_dx);

    // 计算角度差并规范化到[-π, π]
    float delta_angle = curr_angle - prev_angle;
    if (delta_angle > M_PI) {
        delta_angle -= 2 * M_PI;
    }
    else if (delta_angle < -M_PI) {
        delta_angle += 2 * M_PI;
    }

    // 判断方向：负值表示顺时针
    return delta_angle < 0;

}



//处理角度连续性
void Detector::anglesProcessing(std::vector<double>& angles)
{
    for(size_t i=1;i<angles.size();i++){
        double delta=angles[i]-angles[i-1];
        if(delta>CV_PI)
        {
           angles[i]-=2*CV_PI;
        }
        else if(delta<-CV_PI)
        {
           angles[i]+=2*CV_PI;
        }
    }
}



// 判断是否匀速运动  轨迹点 圆心 dt=1/fps 标准差值(噪声水平调整)
bool Detector::SpeedState(double dt,double stdThreshold){

    std::vector<double>angles;
    // 计算各点角度
    for(const auto& point:this->tracking_points)
    {
        double dx=point.x-fit_center.x;
        double dy=point.y-fit_center.y;
        angles.push_back(atan2(dy,dx));
    }
    // 处理角度连续性
    anglesProcessing(angles);

    // 计算角速度
    std::vector<double>omega;
    for(size_t i=1;i<angles.size();i++){
        double d_theta=angles[i]-angles[i-1];
        omega.push_back(d_theta/dt);
    }

    // 计算均值和标准差
    double mean=0.0, stdDev=0.0;
    for(double w:omega){
        mean+=w;
    }
    mean/=omega.size();
    for(double w: omega){
        stdDev+=pow(w-mean,2);
    }
    stdDev=sqrt(stdDev/omega.size());

    // 若标准差低于设定阙值 则为匀速
    return stdDev<stdThreshold;
}




bool Detector::SmallBuffPosition()
{
    //const float rotated_direction = this->is_clockwise ? -1.0f : 1.0f;
    const float math_omega = is_clockwise ? -M_PI/3 : M_PI/3;
    std::vector<cv::Point2f> transformed_points;
    transformed_points.reserve(4);

    const cv::Point2f center = this->fit_center;
    for (const auto& point : rect_points){
        // 计算相对位置并处理坐标系反转
        const float dx = point.x - center.x;
        const float dy = -(point.y - center.y);  // 翻转Y轴

        // 计算原始极坐标
        const float current_angle = std::atan2(dy, dx);
        const float point_radius = getDistance(center, point);

        // 调控时间
        const float newAngle=current_angle + math_omega* 0.1f;

        // 转换回笛卡尔坐标
        const float newX = point_radius * std::cos(newAngle);
        const float newY = point_radius * std::sin(newAngle);

        // 重建世界坐标并恢复 Y 轴方向
        transformed_points.emplace_back(
            center.x + newX,
            center.y - newY  // 将 Y 轴重新翻转到原始方向
        );

    }
    this->small_buff_points = transformed_points;

    return true;
}




// 目标外切矩形四个角点预判位置
bool Detector::LargeBuffPosition(){
    const float a = 0.91;
    constexpr float base_value = 2.090f;
    const float b=base_value - a;
    const float rotation_direction = is_clockwise ? -1.0f : 1.0f;
//    const float angle_delta = rotation_direction * (
//        (a / omega) * (1 - std::cos(omega * deltaTime)) +
//        b * deltaTime);

    const float angle_delta=runPredictor(0.1f);
    std::cout<<"angle_delta: "<<angle_delta<<std::endl;
    std::cout<<"theta: "<<theta<<std::endl;
    calAngleSpeed();
    this->angles.emplace_back(theta);
    std::vector<cv::Point2f> transformed_points;
    transformed_points.reserve(4);
    for (const auto& point : this->rect_points) {
        // 计算相对位置并处理坐标系反转
        const float dx = point.x - fit_center.x;
        const float dy = -(point.y - fit_center.y);  // 翻转Y轴
        // 计算原始极坐标
        float current_angle = std::atan2(dy, dx);
        const float point_radius = getDistance(fit_center, point);
        // 应用角度位移
        //std::cout<<"current_angle: "<<current_angle<<std::endl;
        const float newAngle = current_angle + angle_delta;
        // 转换回笛卡尔坐标
        const float newX = point_radius * std::cos(newAngle);
        const float newY = point_radius * std::sin(newAngle);
        // 重建世界坐标并恢复 Y 轴方向
        transformed_points.emplace_back(
            fit_center.x + newX,
            fit_center.y - newY  // 将 Y 轴重新翻转到原始方向
        );
    }
    this->large_buff_points = transformed_points;
    return true;
}


double Detector::runPredictor(double t)
{
    int windowSize = 100;
    AnglePredictor predictor(t,100);
    for(int i =0;i<110;i++)
    {
        double dt = i*t;
        double realAngle = -0.91/1.9*cos(1.9*dt)+(2.09-0.91)*dt+0.1;
        predictor.addAngle(/*this->theta*/realAngle,dt);
    }
    predictor.computeFrequency();
    double nextAngle = predictor.predictNextAngle();
    std::cout << "Predicted ω: " << predictor.getCurrentOmega() << std::endl;
    std::cout << "Next angle: " << nextAngle << std::endl;
    return nextAngle;
}


void Detector::calAngleSpeed()
{
    auto time_diff = abs(currentTime-prevTime);
    //double seconds = duration_cast<std::chrono::duration<double>>(time_diff).count();
    double seconds = std::chrono::duration<double>(time_diff).count();
    //std::cout<<"time_diff:"<<seconds<<std::endl;
    double Time_diff = seconds/ 1000.0;
    if(Time_diff < 0.000001)
         seconds+= 0.005;
    double prevAngle = std::atan2(prev_point.y - fit_center.y,prev_point.x - fit_center.x);
    double currAngle = std::atan2(buff_center.y - fit_center.y,buff_center.x - fit_center.x);

    double deltaTheta = /*abs(*/currAngle - prevAngle/*)*/;
    if(deltaTheta > CV_PI) deltaTheta -= 2 * CV_PI;
    if(deltaTheta < -CV_PI) deltaTheta += 2 * CV_PI;

    this->theta = deltaTheta;
    //std::cout<<seconds<<std::endl; // 0.034 0.032
    //std::cout<<"theta: "<<deltaTheta<<std::endl;
    //if(deltaTheta>1||deltaTheta<0)
        //std::cout<<"\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\"<<std::endl;
    /*angle_speed = *///return deltaTheta/*/seconds*/ /* deltaTime*/;
    //return deltaTheta / deltaTime;
}



cv::Point2f Detector::getCenter(const std::vector<cv::Point2f>& points)
{
    cv::Point2f center(0.0f,0.0f);
    for(const auto& point:points){
        center+=point;
    }
    center.x /= points.size();
    center.y /= points.size();

    return center;
}

// 判断是否完成激活
bool Detector::isComplete(){
    return this->activation_count>=5 ? true : false;
}

// 不用
cv::Mat Detector::fillHole(const cv::Mat&src)
{
    cv::Mat cutMat;
    cv::Mat broadImg = cv::Mat::zeros(src.rows + 2, src.cols + 2, CV_8UC1);
    src.copyTo(broadImg(cv::Range(1, src.rows + 1), cv::Range(1, src.cols + 1)));
    floodFill(broadImg, cv::Point(0, 0), cv::Scalar(255));
    broadImg(cv::Range(1, src.rows + 1), cv::Range(1, src.cols + 1)).copyTo(cutMat);
    return src | (~cutMat);
}

// 欧式距离
float Detector::getDistance(const cv::Point2f &point_1,const cv::Point2f &point_2)
{
    float x = (point_1 - point_2).x;
    float y = (point_1 - point_2).y;
    return sqrt(x * x + y * y);
}

// 最小二乘法拟合圆
int Detector::CircleFitting(double &dRadius)
{
    if (!this->tracking_points.empty())
    {
        int iNum = (int)tracking_points.size();
        if (iNum < 3)	return 1;
        double X1 = 0.0;
        double Y1 = 0.0;
        double X2 = 0.0;
        double Y2 = 0.0;
        double X3 = 0.0;
        double Y3 = 0.0;
        double X1Y1 = 0.0;
        double X1Y2 = 0.0;
        double X2Y1 = 0.0;
        std::vector<cv::Point2d>::iterator iter;
        std::vector<cv::Point2d>::iterator end = tracking_points.end();
        for (iter = tracking_points.begin(); iter != end; ++iter)
        {
            X1 = X1 + (*iter).x;
            Y1 = Y1 + (*iter).y;
            X2 = X2 + (*iter).x * (*iter).x;
            Y2 = Y2 + (*iter).y * (*iter).y;
            X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
            Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
            X1Y1 = X1Y1 + (*iter).x * (*iter).y;
            X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
            X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
        }
        double C = 0.0;
        double D = 0.0;
        double E = 0.0;
        double G = 0.0;
        double H = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        C = iNum * X2 - X1 * X1;
        D = iNum * X1Y1 - X1 * Y1;
        E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
        G = iNum * Y2 - Y1 * Y1;
        H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);
        c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
        double A = 0.0;
        double B = 0.0;
        double R = 0.0;
        A = a / (-2);
        B = b / (-2);
        R = double(sqrt(a * a + b * b - 4 * c) / 2);
        this->fit_center.x = A;
        this->fit_center.y = B;
        dRadius = R;
        return 0;
    }
    else
        return 1;
    return 0;
}


cv::Point2f Detector::GetBuffCenter()const{
    return buff_center;
}

std::vector<cv::Point2f> Detector::GetRectPoints()const{
    return rect_points;
}

std::vector<cv::Point2f> Detector::GetLargeBuffPoints()const{
    return large_buff_points;
}

cv::Point2f Detector::GetLargeBuffPointsCenter()const{
    return large_buff_points_center;
}

std::vector<cv::Point2f> Detector::GetSmallBuffPoints()const{
    return small_buff_points;
}

cv::Point2f Detector::GetSmallBuffPointsCenter()const{
    return small_buff_points_center;
}

bool Detector::GetMode()const{
    return ModeType;
}

