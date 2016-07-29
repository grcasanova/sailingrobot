#include "colorDetectionNode.h"
#include "logger/Logger.h"
using namespace cv;
using namespace std;

int colorDetectionNode::m_hsvDiff=10;
int colorDetectionNode::m_iLowH = 0;
int colorDetectionNode::m_iHighH = 179;
int colorDetectionNode::m_iLowS = 0;
int colorDetectionNode::m_iHighS = 255;
int colorDetectionNode::m_iLowV = 0;
int colorDetectionNode::m_iHighV = 255;
int colorDetectionNode::m_iColor = 0;
int colorDetectionNode::m_numberOfColorsToTrack=0;
cv::Mat colorDetectionNode::m_imgOriginal;
cv::Mat colorDetectionNode::m_trackBarHSV;
std::vector<std::vector<int> > colorDetectionNode::m_hsvValues;

colorDetectionNode::colorDetectionNode(MessageBus& msgBus,std::vector<string> colors_input)
	: ActiveNode(NodeID::ColorDetection, msgBus),m_minAreaToDetect(2000),
	m_maxAreaToDetect(20000),m_numberOfCapturesPerDetection(5)
{
	m_delay=5000;
	m_Initialised=false;
	vector<string> colors;
	colorDetectionNode::m_trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));

	int numberOfColorInput=(int)colors.size();
	m_inputColorError=false;
	for(int i = 1; i< numberOfColorInput; i++){
        if(colors_input[i].compare("red")==0 ||colors_input[i].compare("orange")==0
		||colors_input[i].compare("yellow")==0 ||colors_input[i].compare("green")==0
		||colors_input[i].compare("blue")==0 ||colors_input[i].compare("purple")==0){
            colors.push_back(colors_input[i]);
            colorDetectionNode::m_numberOfColorsToTrack++;
        }
        else{
			Logger::error("%sWrong color input, at least one color doesn't exists", __PRETTY_FUNCTION__);
			Logger::error("Error put valid colors as an input : ");
			Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
			m_inputColorError=true;
        }
    }
    if(colorDetectionNode::m_numberOfColorsToTrack==0){
		Logger::error("%sWrong color input, you have to give at least one color input", __PRETTY_FUNCTION__);
		Logger::error("Error put valid colors as an input : ");
		Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
        m_inputColorError=true;
    }

	colorDetectionNode::m_hsvValues=findHsvTreshold(colors, m_colorDrawing);
}

colorDetectionNode::colorDetectionNode(MessageBus& msgBus,int port, int delay,std::vector<string> colors_input)
	: ActiveNode(NodeID::ColorDetection, msgBus),m_minAreaToDetect(2000),
	m_maxAreaToDetect(20000),m_numberOfCapturesPerDetection(5)
{
	m_delay=delay;
	m_port=port;
	m_Initialised=false;
	vector<string> colors;
	colorDetectionNode::m_trackBarHSV = Mat3b(100, 300, Vec3b(0,0,0));

	int numberOfColorInput=(int)colors.size();
	m_inputColorError=false;
	for(int i = 1; i< numberOfColorInput; i++){
        if(colors_input[i].compare("red")==0 ||colors_input[i].compare("orange")==0
		||colors_input[i].compare("yellow")==0 ||colors_input[i].compare("green")==0
		||colors_input[i].compare("blue")==0 ||colors_input[i].compare("purple")==0){
            colors.push_back(colors_input[i]);
            colorDetectionNode::m_numberOfColorsToTrack++;
        }
        else{
			Logger::error("%sWrong color input, at least one color doesn't exists", __PRETTY_FUNCTION__);
			Logger::error("Error put valid colors as an input : ");
			Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
			colorDetectionNode::m_inputColorError=true;
        }
    }
    if(colorDetectionNode::colorDetectionNode::m_numberOfColorsToTrack==0){
		Logger::error("%sWrong color input, you have to give at least one color input", __PRETTY_FUNCTION__);
		Logger::error("Error put valid colors as an input : ");
		Logger::error("red\norange\nyellow\ngreen\nblue\npurple");
        m_inputColorError=true;
    }
	colorDetectionNode::m_hsvValues=findHsvTreshold(colors, m_colorDrawing);
}

bool colorDetectionNode::init(){
	initHsvColors();
	VideoCapture cap(m_port);
	m_cap=cap; //capture the video from webcam
    if ( !m_cap.isOpened() )  // if not success, exit program
    {
		Logger::error("%sCannot open the webcam, check usb port", __PRETTY_FUNCTION__);
        m_Initialised=false;
    }
	else{
		m_Initialised=true;

	}
	return m_Initialised;

}

void colorDetectionNode::start()
{
	if(m_Initialised && !m_inputColorError)
	{
		runThread(colorDetectionThreadFunc);
	}
	else if(!m_Initialised)
	{
		Logger::error("%s Cannot start colorDetectionNode thread as the node was not correctly initialised!", __PRETTY_FUNCTION__);
	}
	else if(m_inputColorError)
	{
		Logger::error("%sCannot start colorDetectionNode thread because of a wrong color input!", __PRETTY_FUNCTION__);
	}
}



void colorDetectionNode::changecolorTrackbar(){
    Mat HSVmat=  Mat::zeros(1,1 , CV_8UC3 );
    Mat RGBmat=  Mat::zeros(1,1 , CV_8UC3 );
    cv::Vec3b pixel ;
    pixel[0] = (int)(colorDetectionNode::m_iLowH+colorDetectionNode::m_iHighH)/2;
    pixel[1] = (int)(colorDetectionNode::m_iLowS+colorDetectionNode::m_iHighS)/2;
    pixel[2] = (int)(colorDetectionNode::m_iLowV+colorDetectionNode::m_iHighV)/2;
    HSVmat.at<Vec3b>(0,0)=pixel;
    cvtColor(HSVmat, RGBmat,CV_HSV2RGB);
    pixel=RGBmat.at<Vec3b>(0,0);
    colorDetectionNode::m_trackBarHSV.setTo(Scalar(pixel[2],pixel[1],pixel[0]));
    imshow("Change Detection Values",colorDetectionNode::m_trackBarHSV);
}

void colorDetectionNode::update_trackbar_hsv_values( int, void* )
{
    colorDetectionNode::m_iLowH = colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][0];
    colorDetectionNode::m_iHighH =colorDetectionNode:: m_hsvValues[colorDetectionNode::m_iColor][3];
    colorDetectionNode::m_iLowS = colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][1];
    colorDetectionNode::m_iHighS = colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][4];
    colorDetectionNode::m_iLowV = colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][2];
    colorDetectionNode::m_iHighV = colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][5];
    setTrackbarPos("LowH","Change Detection Values",colorDetectionNode::m_iLowH);
    setTrackbarPos("LowS","Change Detection Values",colorDetectionNode::m_iLowS);
    setTrackbarPos("LowV","Change Detection Values",colorDetectionNode::m_iLowV);
    setTrackbarPos("HighH","Change Detection Values",colorDetectionNode::m_iHighH);
    setTrackbarPos("HighS","Change Detection Values",colorDetectionNode::m_iHighS);
    setTrackbarPos("HighV","Change Detection Values",colorDetectionNode::m_iHighV);
    changecolorTrackbar();
}

void colorDetectionNode::get_trackbar_hsv_values( int, void* )
{
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][0]=getTrackbarPos("LowH", "Change Detection Values");
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][1]=getTrackbarPos("LowS", "Change Detection Values");
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][2]=getTrackbarPos("LowV", "Change Detection Values");
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][3]=getTrackbarPos("HighH", "Change Detection Values");
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][4]=getTrackbarPos("HighS", "Change Detection Values");
    colorDetectionNode::m_hsvValues[colorDetectionNode::m_iColor][5]=getTrackbarPos("HighV", "Change Detection Values");
    changecolorTrackbar();
}
void colorDetectionNode::get_on_click_hsv_pixel_values( int event, int x, int y, int, void* )
{
    if( event == EVENT_LBUTTONDOWN ){
        Mat HSV;
        cvtColor(colorDetectionNode::m_imgOriginal, HSV,CV_BGR2HSV);
        Vec3b pixel = HSV.at<Vec3b>(y,x);
        colorDetectionNode::m_iLowH =(pixel[0]-colorDetectionNode::m_hsvDiff);colorDetectionNode::
        colorDetectionNode::m_iHighH =(pixel[0]+colorDetectionNode::m_hsvDiff);
        colorDetectionNode::m_iLowS = (pixel[1]-colorDetectionNode::m_hsvDiff);
        colorDetectionNode::m_iHighS = (pixel[1]+colorDetectionNode::m_hsvDiff);
        colorDetectionNode::m_iLowV = (pixel[2]-colorDetectionNode::m_hsvDiff);
        colorDetectionNode::m_iHighV = (pixel[2]+colorDetectionNode::m_hsvDiff);
        setTrackbarPos("LowH","Change Detection Values",colorDetectionNode::m_iLowH);
        setTrackbarPos("LowS","Change Detection Values",colorDetectionNode::m_iLowS);
        setTrackbarPos("LowV","Change Detection Values",colorDetectionNode::m_iLowV);
        setTrackbarPos("HighH","Change Detection Values",colorDetectionNode::m_iHighH);
        setTrackbarPos("HighS","Change Detection Values",colorDetectionNode::m_iHighS);
        setTrackbarPos("HighV","Change DeteccolorDetectionNode::tion Values",colorDetectionNode::m_iHighV);
        changecolorTrackbar();
    }
}
void colorDetectionNode::initWindowsAndTrackbars(){
    namedWindow("Change Detection Values", CV_WINDOW_NORMAL); //create a window called "Change Detection Values"

    if(colorDetectionNode::m_numberOfColorsToTrack>1){
        colorDetectionNode::m_numberOfColorsToTrack=colorDetectionNode::m_numberOfColorsToTrack-1;
        createTrackbar("Color", "Change Detection Values", &colorDetectionNode::m_iColor,colorDetectionNode:: m_numberOfColorsToTrack,update_trackbar_hsv_values);
    }
    colorDetectionNode::m_iLowH =colorDetectionNode::m_hsvValues[0][0];
    colorDetectionNode::m_iHighH = colorDetectionNode::m_hsvValues[0][3];
    colorDetectionNode::m_iLowS = colorDetectionNode::m_hsvValues[0][1];
    colorDetectionNode::m_iHighS = colorDetectionNode::m_hsvValues[0][4];
    colorDetectionNode::m_iLowV = colorDetectionNode::m_hsvValues[0][2];
    colorDetectionNode::m_iHighV = colorDetectionNode::m_hsvValues[0][5];
    createTrackbar("m_minAreaToDetect", "Change Detection Values", &m_minAreaToDetect,m_maxAreaToDetect);

    //Create trackbars in "Change Detection Values" window
    createTrackbar("LowH", "Change Detection Values", &m_iLowH, 179, get_trackbar_hsv_values); //Hue (0 - 179)
    createTrackbar("HighH", "Change Detection Values", &m_iHighH, 179, get_trackbar_hsv_values);

    createTrackbar("LowS", "Change Detection Values", &m_iLowS, 255, get_trackbar_hsv_values); //Saturation (0 - 255)
    createTrackbar("HighS", "Change Detection Values", &m_iHighS, 255, get_trackbar_hsv_values);

    createTrackbar("LowV", "Change Detection Values", &m_iLowV, 255, get_trackbar_hsv_values);//Value (0 - 255)
    createTrackbar("HighV", "Change Detection Values", &m_iHighV, 255, get_trackbar_hsv_values);
    changecolorTrackbar();
    moveWindow("Change Detection Values", 0,0);

    namedWindow("Thresholded Image", CV_WINDOW_NORMAL);
    namedWindow("Detection", CV_WINDOW_NORMAL);
    resizeWindow("Thresholded Image", 500, 500);
    moveWindow("Thresholded Image", 700,0);
    resizeWindow("Detection", 500, 500);
    moveWindow("Detection", 1300,0);
}



void colorDetectionNode::colorDetectionThreadFunc(void* nodePtr)
{
	colorDetectionNode* node = (colorDetectionNode*)nodePtr;
	Logger::info("colorDetectionNode thread started");

    Mat imgThresholded;
    vector<Point2f> mc;
    vector<Rect> rotated_bounding_rects;
    vector<Mat> imgsThresholded (colorDetectionNode::m_numberOfColorsToTrack);
    node->initWindowsAndTrackbars();
    vector<vector<Rect> > rotated_bounding_rects_several_captures(colorDetectionNode::m_numberOfColorsToTrack);
    vector<vector<Rect> > rotated_bounding_rects_merged_list(colorDetectionNode::m_numberOfColorsToTrack);
    vector<vector<Point> > centers(colorDetectionNode::m_numberOfColorsToTrack);
    std::vector<ObstacleData> obstacles;

    while (true)
    {
        for(int h = 0; h<(int)(node->m_numberOfCapturesPerDetection);h++){
            bool bSuccess = node->m_cap.read(colorDetectionNode::m_imgOriginal); // read a new frame from video
            if (!bSuccess) { //if not success, break loop
				Logger::warning("%sCannot read a frame from video stream!", __PRETTY_FUNCTION__);
				continue;
            }
            for(int i = 0; i<(int)colorDetectionNode::m_hsvValues.size(); i++){
                imgThresholded=threshold(colorDetectionNode::m_imgOriginal,colorDetectionNode::m_hsvValues[i]);
                morphologicalOperations (imgThresholded);
                computeContoursCentersRectangles(imgThresholded,mc,rotated_bounding_rects,node->m_minAreaToDetect );
                //drawCentersRectangles(m_imgOriginal, mc,rotated_bounding_rects, colorDrawing[i]);
                rotated_bounding_rects_several_captures[i].insert(rotated_bounding_rects_several_captures[i].begin(),rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                rotated_bounding_rects.erase(rotated_bounding_rects.begin(),rotated_bounding_rects.end());
                imgsThresholded[i]=imgThresholded;
            }
        }
        for( int i = 0; i<(int)rotated_bounding_rects_several_captures.size(); i++ ){
            //supressSmallRectangles(rotated_bounding_rects_several_captures[i], m_minAreaToDetect);
            rotated_bounding_rects_merged_list[i] = compareRects(colorDetectionNode::m_imgOriginal,rotated_bounding_rects_several_captures[i]);
            centers[i]=findRectanglesCenters(rotated_bounding_rects_merged_list[i]);
            for(int j = 0; j <(int)rotated_bounding_rects_merged_list[i].size(); j++){
                rectangle(colorDetectionNode::m_imgOriginal, rotated_bounding_rects_merged_list[i][j],node->m_colorDrawing[i] ,4, 8,0);
                circle( colorDetectionNode::m_imgOriginal, centers[i][j], 10,node->m_colorDrawing[i] , 4, 8, 0 );
            }
        }
        //cout << rotated_bounding_rects_merged_list.size() << endl;
        computeObstaclesAnglePosition(colorDetectionNode::m_imgOriginal, obstacles, rotated_bounding_rects_merged_list );
        printObstacleVector(obstacles);
        rotated_bounding_rects_several_captures.erase(rotated_bounding_rects_several_captures.begin(),rotated_bounding_rects_several_captures.end());
        rotated_bounding_rects_several_captures.resize(colorDetectionNode::m_numberOfColorsToTrack);
        obstacles.erase(obstacles.begin(),obstacles.end());

        imshow("Thresholded Image", imgsThresholded[colorDetectionNode::m_iColor]); //show the thresholded image
        imshow("Detection", colorDetectionNode::m_imgOriginal); //show the original image
        setMouseCallback( "Detection", get_on_click_hsv_pixel_values, 0 );
		/*
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
		*/
        waitKey(node->m_delay);
    }

}

void colorDetectionNode::processMessage(const Message* msgPtr)
{

}