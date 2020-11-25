// -*- C++ -*-
/*!
 * @file  KnottingMotionGenerator.cpp
 * @brief Arm Image Generator RT Component
 * @date $Date$
 *
 * $Id$
 */

#include "KnottingMotionGenerator.h"

#include <iomanip>
//#include <fstream>
#include <ctime>


#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif


// Module specification
// <rtc-template block="module_spec">
static const char* armimagegenerator_spec[] =
  {
    "implementation_id", "KnottingMotionGenerator",
    "type_name",         "KnottingMotionGenerator",
    "description",       "Arm Image Generator RT Component",
    "version",           "1.0.0",
    "vendor",            "ogata_lab",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "1",
    "conf.default.j0max", "1.57076",
    "conf.default.j1max", "1.57076",
    "conf.default.j0min", "-1.57076",
    "conf.default.j1min", "-1.57076",
    "conf.default.j0step", "0.157076",
    "conf.default.j1step", "0.157076",
	"conf.default.wait_interval", "1.5",
    "conf.default.camera_wait_time", "3.0",
    "conf.default.gripper_close_ratio", "0.1",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.j0max", "text",
    "conf.__widget__.j1max", "text",
    "conf.__widget__.j0min", "text",
    "conf.__widget__.j1min", "text",
    "conf.__widget__.j0step", "text",
    "conf.__widget__.j1step", "text",
    
    "conf.__widget__.gripper_close_ratio", "slider.0.1",
    // Constraints
    "conf.__constraints__.gripper_close_ratio", "0.0<=x<=1.0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
KnottingMotionGenerator::KnottingMotionGenerator(RTC::Manager* manager)
// <rtc-template block="initializer">
: RTC::DataFlowComponentBase(manager),
m_inIn("in", m_in),
m_gripperOCOut("gripperOC", m_gripperOC),  
m_cameraIn("camera", m_camera),
m_manipCommon_LPort("manipCommon_L"),
m_manipMiddle_LPort("manipMiddle_L"),
m_manipCommon_RPort("manipCommon_R"),
m_manipMiddle_RPort("manipMiddle_R")

// </rtc-template>
, m_jointPos(new JARA_ARM::JointPos())
, m_LjointPos(new JARA_ARM_LEFT::JointPos())
{
}

/*!
 * @brief destructor
 */
KnottingMotionGenerator::~KnottingMotionGenerator()
{
}



RTC::ReturnCode_t KnottingMotionGenerator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  addInPort("camera", m_cameraIn);

  //gripperOC=gripper open or close
  addOutPort("gripperOC", m_gripperOCOut);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_manipCommon_LPort.registerConsumer("JARA_ARM_LEFT_ManipulatorCommonInterface_Common", "JARA_ARM_LEFT::ManipulatorCommonInterface_Common", m_manipCommon_L);
  m_manipMiddle_LPort.registerConsumer("JARA_ARM_LEFT_ManipulatorCommonInterface_Middle", "JARA_ARM_LEFT::ManipulatorCommonInterface_Middle", m_manipMiddle_L);
  m_manipCommon_RPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_manipCommon_R);
  m_manipMiddle_RPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_manipMiddle_R);
  
  // Set CORBA Service Ports
  addPort(m_manipCommon_LPort);
  addPort(m_manipMiddle_LPort);
  addPort(m_manipCommon_RPort);
  addPort(m_manipMiddle_RPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "1");
  bindParameter("j0max", m_j0max, "1.57076");
  bindParameter("j1max", m_j1max, "1.57076");
  bindParameter("j0min", m_j0min, "-1.57076");
  bindParameter("j1min", m_j1min, "-1.57076");
  bindParameter("j0step", m_j0step, "0.157076");
  bindParameter("j1step", m_j1step, "0.157076");
  bindParameter("wait_interval", m_wait_interval, "1.5");
  bindParameter("camera_wait_time", m_camera_wait_time, "3.0");
  bindParameter("gripper_close_ratio", m_gripper_close_ratio, "0.1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t KnottingMotionGenerator::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t KnottingMotionGenerator::onActivated(RTC::UniqueId ec_id)
{
	std::cout << "[KnottingMotionGenerator] Initializing Arm and Parameters" << std::endl;

	coil::TimeValue tv1(5.0);
	coil::sleep(tv1);

	std::cout << "[KnottingMotionGenerator] Waiting Arm Component is Activated....." << std::endl;
	while (true) {
      //int ok_count = 0;
 		const RTC::PortProfile& pp = m_manipCommon_LPort.getPortProfile();
 		if (pp.connector_profiles.length() > 0) {
 			RTC::PortProfile_var pp0 = pp.connector_profiles[0].ports[0]->get_port_profile();
 			RTC::ComponentProfile_var cp0 = pp0->owner->get_component_profile();
 			if (std::string(cp0->type_name) == armimagegenerator_spec[1]) {
 				RTC::PortProfile_var pp1 = pp.connector_profiles[0].ports[1]->get_port_profile();
 				RTC::ComponentProfile_var cp1 = pp1->owner->get_component_profile();
 				RTC::ExecutionContext_var ec1 = pp1->owner->get_context(0);
 				if (ec1->get_component_state(pp1->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 			else {
 				RTC::ExecutionContext_var ec0 = pp0->owner->get_context(0);
 				if (ec0->get_component_state(pp0->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 		}

		const RTC::PortProfile& qq = m_manipCommon_RPort.getPortProfile();
 		if (qq.connector_profiles.length() > 0) {
 			RTC::PortProfile_var qq0 = qq.connector_profiles[0].ports[0]->get_port_profile();
 			RTC::ComponentProfile_var cq0 = qq0->owner->get_component_profile();
 			if (std::string(cq0->type_name) == armimagegenerator_spec[1]) {
 				RTC::PortProfile_var qq1 = qq.connector_profiles[0].ports[1]->get_port_profile();
 				RTC::ComponentProfile_var cq1 = qq1->owner->get_component_profile();
 				RTC::ExecutionContext_var ecq1 = qq1->owner->get_context(0);
 				if (ecq1->get_component_state(qq1->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 			else {
 				RTC::ExecutionContext_var ecq0 = qq0->owner->get_context(0);
 				if (ecq0->get_component_state(qq0->owner) == ACTIVE_STATE) {
 					break;
 				}
 			}
 		}
 	}

	m_gripperOC.data.length(2);
        m_gripperOC.data[0]=0;
        m_gripperOC.data[1]=0;

        m_gripperOCOut.write();
	

 	m_manipCommon_L->servoON();
 	m_manipMiddle_L->setSpeedJoint(20);
 	m_manipCommon_R->servoON();
 	m_manipMiddle_R->setSpeedJoint(20);

 	m_jointPos->length(6);
	m_jointPos[0] = -M_PI/2.27;
 	m_jointPos[1] = -M_PI/1.42;
 	m_jointPos[2] = -M_PI/1.33;
 	m_jointPos[3] = M_PI/61.86;
 	m_jointPos[4] = -M_PI/0.79;
 	m_jointPos[5] = M_PI/1.76;

 	m_manipMiddle_R->movePTPJointAbs(m_jointPos);


	m_LjointPos->length(6);
	m_LjointPos[0] = M_PI/2.07;
 	m_LjointPos[1] = -M_PI/3.00;
 	m_LjointPos[2] = M_PI/1.35;
 	m_LjointPos[3] = -M_PI/1.12;
 	m_LjointPos[4] = -M_PI/1.29;
 	m_LjointPos[5] = -M_PI/1.98;

	m_manipMiddle_L->movePTPJointAbs(m_LjointPos);

 	coil::TimeValue tv(3.0);
 	coil::sleep(tv);


 	m_sleepTime = coil::TimeValue(m_wait_interval);
 	std::cout << "[KnottingMotionGenerator] Wait " << m_sleepTime.sec() << "[sec], " << m_sleepTime.usec() << "[usec]" << std::endl;
	
 	std::cout << "[KnottingMotionGenerator] Ready." << std::endl;

	State=true;

	
 	return RTC::RTC_OK;
 }



 RTC::ReturnCode_t KnottingMotionGenerator::onDeactivated(RTC::UniqueId ec_id)
 {


 	coil::TimeValue tv(3.0);
 	coil::sleep(tv);

 	m_manipCommon_L->servoOFF();
 	m_manipCommon_R->servoOFF();
 	return RTC::RTC_OK;
 }

 double Uniform( void ){
   return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
 }

 void CarPosInit(JARA_ARM::CarPosWithElbow targetPos){
   targetPos.elbow = 0;
   targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
   targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
   targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;
 }

 RTC::ReturnCode_t KnottingMotionGenerator::onExecute(RTC::UniqueId ec_id)
 {



   // std::cout << "--------------------------------------------------" << std::endl;

   //JARA_ARM::CarPosWithElbow targetPos;
  
  JARA_ARM::CarPosWithElbow_var pos = new JARA_ARM::CarPosWithElbow();
  JARA_ARM::JointPos_var jpos = new JARA_ARM::JointPos();


  m_gripperOC.data.length(2);
  m_gripperOC.data[0]=0;
  m_gripperOC.data[1]=0;

  m_gripperOCOut.write();
  
  std::cout << "[KnottingMotionGenerator] Right_ Ready" << std::endl;

        m_jointPos[0] = -M_PI/2.27;
 	m_jointPos[1] = -M_PI/1.42;
 	m_jointPos[2] = -M_PI/1.33;
 	m_jointPos[3] = M_PI/61.86;
 	m_jointPos[4] = -M_PI/0.79;
 	m_jointPos[5] = M_PI/1.76;

  m_manipMiddle_R->movePTPJointAbs(m_jointPos);

  m_manipCommon_R->getFeedbackPosJoint(jpos);
  std::cout << jpos[0] << ", "
	    << jpos[1] << ", "
	    << jpos[2] << ", "
	    << jpos[3] << ", "
	    << jpos[4] << ", "
	    << jpos[5] << ", "
	    << std::endl;
  
  coil::sleep(5.0);

  std::cout << "[KnottingMotionGenerator] Left_ Ready" << std::endl;

   	m_LjointPos[0] = M_PI/2.07;
 	m_LjointPos[1] = -M_PI/3.00;
 	m_LjointPos[2] = M_PI/1.35;
 	m_LjointPos[3] = -M_PI/1.12;
 	m_LjointPos[4] = -M_PI/1.29;
 	m_LjointPos[5] = -M_PI/1.98;

  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(5.0);


  std::cout << "[KnottingMotionGenerator] hidari_1" << std::endl;
  	m_LjointPos[0] = M_PI/2.07;
 	m_LjointPos[1] = -M_PI/3.00;
 	m_LjointPos[2] = M_PI/1.35;
 	m_LjointPos[3] = -M_PI/1.12;
 	m_LjointPos[4] = -M_PI/1.29;
 	m_LjointPos[5] = -M_PI/1.02;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] migi_1" << std::endl;
  	m_jointPos[0] = -M_PI/2.46;
  	m_jointPos[1] = -M_PI/1.32;
  	m_jointPos[2] = -M_PI/1.32;
  	m_jointPos[3] = M_PI/9.74;
  	m_jointPos[4] = -M_PI/0.79;
  	m_jointPos[5] = M_PI/1.63;
	
  m_manipMiddle_R->movePTPJointAbs(m_jointPos); 
  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] hidari_2" << std::endl;
  	m_LjointPos[0] = M_PI/2.32;
 	m_LjointPos[1] = -M_PI/4.10;
 	m_LjointPos[2] = M_PI/1.36;
 	m_LjointPos[3] = -M_PI/0.97;
 	m_LjointPos[4] = -M_PI/1.29;
 	m_LjointPos[5] = -M_PI/0.97;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] Gripper Closed." << std::endl;
  m_manipMiddle_L->closeGripper();

  m_gripperOC.data[1]=1;
  m_gripperOCOut.write();
  coil::sleep(m_sleepTime);

  coil::sleep(3.0);
  
  std::cout << "[KnottingMotionGenerator] migi_2" << std::endl;
  	m_jointPos[0] = -M_PI/2.21;
  	m_jointPos[1] = -M_PI/1.24;
  	m_jointPos[2] = -M_PI/1.35;
  	m_jointPos[3] = M_PI/11.12;
  	m_jointPos[4] = -M_PI/0.80;
  	m_jointPos[5] = M_PI/1.81;

  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);
  
  // JARA_ARM::CarPosWithElbow targetPos;
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;


  //CarPosInit(targetPos);
  std::cout << "move backward" << std::endl;
        targetPos.carPos[0][3] = -0.015;
        targetPos.carPos[2][3] = +0.015;

  m_manipMiddle_R->movePTPCartesianRel(targetPos);
  coil::sleep(2.0);


  count = 0;
  Choucho = 0;
  
  while(State==true){
    std::cout << "loop loop loop."<< std::endl;
    targetPos.elbow = 0;
    targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
    targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
    targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;
    if(m_inIn.isNew()) {
      float sum = 0;
      m_inIn.read();
      // for(int i = 0;i < m_in.data.length();i++) {
      // 	sum += m_in.data[i];
      // }
      // if(sum <= 26.0){

      sum += m_in.data[0]*100;
      sum += m_in.data[3]*100;
      if(sum <= 960.0){
  	std::cout << "Under 960.0, Move ur5e: "<< sum << std::endl;
	Choucho = sum;
  	State = false;
  	break;
      }else{
  	std::cout << "Non."<< std::endl;
  	std::cout << "Test: move right" << std::endl;
  	targetPos.carPos[1][3] = -0.003;
  	if(count%3==0){
  	  targetPos.carPos[0][3] = -0.0015;
	  targetPos.carPos[2][3] = +0.0015;
  	  std::cout << "back" << std::endl; 
  	}
  	m_manipMiddle_R->movePTPCartesianRel(targetPos);
  	count++;
  	coil::sleep(2.0);
      }
    }
  }

  coil::sleep(2.0);
  //CarPosInit(targetPos);
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;
  std::cout << "move left." << std::endl;
        targetPos.carPos[1][3] = +0.05;

  m_manipMiddle_R->movePTPCartesianRel(targetPos);
  coil::sleep(2.0);

  //CarPosInit(targetPos);
  targetPos.elbow = 0;
  targetPos.carPos[0][0] = 1; targetPos.carPos[0][1] = 0; targetPos.carPos[0][2] = 0; targetPos.carPos[0][3] = 0;
  targetPos.carPos[1][0] = 0; targetPos.carPos[1][1] = 1; targetPos.carPos[1][2] = 0; targetPos.carPos[1][3] = 0;
  targetPos.carPos[2][0] = 0; targetPos.carPos[2][1] = 0; targetPos.carPos[2][2] = 1; targetPos.carPos[2][3] = 0;
  if(960 > Choucho && Choucho > 940){
    std::cout << "Large move forward" << std::endl;
    targetPos.carPos[0][3] = +0.02;
    targetPos.carPos[2][3] = -0.02;

  }else{
    std::cout << "Small move forward" << std::endl;
    targetPos.carPos[0][3] = +0.01;
    targetPos.carPos[2][3] = -0.01;
  }
  m_manipMiddle_R->movePTPCartesianRel(targetPos);
  coil::sleep(2.0);


  std::cout << "test" << std::endl;
  m_manipCommon_R->getFeedbackPosJoint(jpos);
  std::cout << jpos[0] << ", "
	    << jpos[1] << ", "
	    << jpos[2] << ", "
	    << jpos[3] << ", "
	    << jpos[4] << ", "
	    << jpos[5] << ", "
	    << std::endl;
    
  
  std::cout << "[KnottingMotionGenerator] migi_3" << std::endl;
        m_jointPos[0] = -M_PI/2.19;
  	m_jointPos[1] = -M_PI/1.26;
  	m_jointPos[2] = -M_PI/1.34;
  	m_jointPos[3] = M_PI/11.46;
  	m_jointPos[4] = -M_PI/0.80;
  	m_jointPos[5] = M_PI/1.42;

	
  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);
  

  std::cout << "[KnottingMotionGenerator] Gripper Closed." << std::endl;
  m_manipMiddle_R->closeGripper();

  m_gripperOC.data[0]=1;
  m_gripperOCOut.write();
  coil::sleep(m_sleepTime);
  

  coil::sleep(3.0);
  
  std::cout << "[KnottingMotionGenerator] migi_4" << std::endl;
  	m_jointPos[0] = -M_PI/1.93;
  	m_jointPos[1] = -M_PI/1.41;
  	m_jointPos[2] = -M_PI/1.34;
  	m_jointPos[3] = -M_PI/15.72;
  	m_jointPos[4] = -M_PI/0.80;
  	m_jointPos[5] = M_PI/1.56;
	
  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] hidari_3" << std::endl;
  	m_LjointPos[0] = M_PI/2.17;
  	m_LjointPos[1] = -M_PI/3.38;
  	m_LjointPos[2] = M_PI/1.35;
  	m_LjointPos[3] = -M_PI/1.04;
  	m_LjointPos[4] = -M_PI/1.29;
  	m_LjointPos[5] = -M_PI/0.76;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] hidari_4" << std::endl;
  	m_LjointPos[0] = M_PI/2.12;
  	m_LjointPos[1] = -M_PI/3.33;
  	m_LjointPos[2] = M_PI/1.35;
  	m_LjointPos[3] = -M_PI/1.07;
  	m_LjointPos[4] = -M_PI/1.50;
  	m_LjointPos[5] = -M_PI/0.71;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);

  // std::cout << "[KnottingMotionGenerator] migi_5" << std::endl;
  // 	m_jointPos[0] = -M_PI/2.19;
  // 	m_jointPos[1] = -M_PI/1.52;
  // 	m_jointPos[2] = -M_PI/1.30;
  // 	m_jointPos[3] = -M_PI/18.63;
  // 	m_jointPos[4] = -M_PI/0.73;
  // 	m_jointPos[5] = M_PI/1.40;
	
  // m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  // coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] migi_6" << std::endl;
  	m_jointPos[0] = -M_PI/2.31;
  	m_jointPos[1] = -M_PI/1.77;
  	m_jointPos[2] = -M_PI/1.31;
  	m_jointPos[3] = -M_PI/6.87;
  	m_jointPos[4] = -M_PI/0.73;
  	m_jointPos[5] = M_PI/1.35;
	
  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] hidari_5" << std::endl;
  	m_LjointPos[0] = M_PI/2.21;
  	m_LjointPos[1] = -M_PI/2.27;
  	m_LjointPos[2] = M_PI/1.35;
  	m_LjointPos[3] = -M_PI/1.25;
  	m_LjointPos[4] = -M_PI/1.50;
  	m_LjointPos[5] = -M_PI/0.62;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] migi_7" << std::endl;
  	m_jointPos[0] = -M_PI/2.11;
  	m_jointPos[1] = -M_PI/1.91;
  	m_jointPos[2] = -M_PI/1.36;
  	m_jointPos[3] = -M_PI/4.41;
  	m_jointPos[4] = -M_PI/0.73;
  	m_jointPos[5] = M_PI/1.44;
	
  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);

   std::cout << "[KnottingMotionGenerator] hidari_6" << std::endl;
  	m_LjointPos[0] = M_PI/2.04;
  	m_LjointPos[1] = -M_PI/2.09;
  	m_LjointPos[2] = M_PI/1.40;
  	m_LjointPos[3] = -M_PI/1.40;
  	m_LjointPos[4] = -M_PI/1.50;
  	m_LjointPos[5] = -M_PI/0.63;
	
  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);


  std::cout << "[KnottingMotionGenerator] Last move migi" << std::endl;

   	m_jointPos[0] = -M_PI/1.66;
  	m_jointPos[1] = -M_PI/1.67;
  	m_jointPos[2] = -M_PI/1.46;
  	m_jointPos[3] = -M_PI/3.92;
  	m_jointPos[4] = -M_PI/0.72;
  	m_jointPos[5] = M_PI/1.79;

  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(3.0);

  std::cout << "[KnottingMotionGenerator] Last move hidari" << std::endl;

   	m_LjointPos[0] = M_PI/1.59;
  	m_LjointPos[1] = -M_PI/2.61;
  	m_LjointPos[2] = M_PI/1.54;
  	m_LjointPos[3] = -M_PI/1.49;
  	m_LjointPos[4] = -M_PI/1.56;
  	m_LjointPos[5] = -M_PI/0.70;

  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(3.0);
  
  std::cout << "[KnottingMotionGenerator] Gripper Opened." << std::endl;
  m_manipMiddle_R->openGripper();

  m_gripperOC.data[0]=0;
  m_gripperOCOut.write();
  coil::sleep(m_sleepTime);

  std::cout << "[KnottingMotionGenerator] Gripper Opened." << std::endl;
  m_manipMiddle_L->openGripper();

  m_gripperOC.data[1]=0;
  m_gripperOCOut.write();
  coil::sleep(m_sleepTime);

  std::cout << "[KnottingMotionGenerator] Ready migi" << std::endl;

   	m_jointPos[0] = -M_PI/2.27;
  	m_jointPos[1] = -M_PI/1.42;
  	m_jointPos[2] = -M_PI/1.33;
  	m_jointPos[3] = M_PI/61.86;
  	m_jointPos[4] = -M_PI/0.79;
  	m_jointPos[5] = M_PI/1.76;

  m_manipMiddle_R->movePTPJointAbs(m_jointPos);  
  coil::sleep(5.0);

  std::cout << "[KnottingMotionGenerator] Ready hidari" << std::endl;

   	m_LjointPos[0] = M_PI/2.07;
  	m_LjointPos[1] = -M_PI/3.00;
  	m_LjointPos[2] = M_PI/1.35;
  	m_LjointPos[3] = -M_PI/1.12;
  	m_LjointPos[4] = -M_PI/1.29;
  	m_LjointPos[5] = -M_PI/1.05;

  m_manipMiddle_L->movePTPJointAbs(m_LjointPos);  
  coil::sleep(5.0);

  std::cout << "------------------------------------------------------------" << std::endl;

  State = true;

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t KnottingMotionGenerator::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t KnottingMotionGenerator::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void KnottingMotionGeneratorInit(RTC::Manager* manager)
  {
    coil::Properties profile(armimagegenerator_spec);
    manager->registerFactory(profile,
                             RTC::Create<KnottingMotionGenerator>,
                             RTC::Delete<KnottingMotionGenerator>);
  }
  
};


