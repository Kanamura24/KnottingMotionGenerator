// -*- C++ -*-
/*!
 * @file  KnottingMotionGenerator.h
 * @brief Arm Image Generator RT Component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ARMIMAGEGENERATOR_H
#define ARMIMAGEGENERATOR_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ManipulatorCommonInterface_CommonStub.h"
#include "ManipulatorCommonInterface_MiddleLevelStub.h"
#include "LeftManipulatorCommonInterface_CommonStub.h"
#include "LeftManipulatorCommonInterface_MiddleLevelStub.h"
#include "ImgStub.h"

// </rtc-template>


#include <opencv2/opencv.hpp>
// Service Consumer stub headers
// <rtc-template block="port_stub_h">
using namespace Img;
// </rtc-template>

using namespace RTC;

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

/*!
 * @class KnottingMotionGenerator
 * @brief Arm Image Generator RT Component
 *
 */
class KnottingMotionGenerator
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  KnottingMotionGenerator(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~KnottingMotionGenerator();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 1
   */
  int m_debug;
  /*!
   * 
   * - Name:  j0max
   * - DefaultValue: 1.57076
   */
  float m_j0max;
  /*!
   * 
   * - Name:  j1max
   * - DefaultValue: 1.57076
   */
  float m_j1max;
  /*!
   * 
   * - Name:  j0min
   * - DefaultValue: -1.57076
   */
  float m_j0min;
  /*!
   * 
   * - Name:  j1min
   * - DefaultValue: -1.57076
   */
  float m_j1min;
  /*!
   * 
   * - Name:  j0step
   * - DefaultValue: 0.157076
   */
  float m_j0step;
  /*!
   * 
   * - Name:  j1step
   * - DefaultValue: 0.157076
   */
  float m_j1step;
  /*!
   * 
   * - Name:  wait_interval
   * - DefaultValue: 1.0
   */
  float m_wait_interval;
  /*!
   * 
   * - Name:  camera_wait_time
   * - DefaultValue: 3.0
   */
  float m_camera_wait_time ;
  /*!
   * 
   * - Name:  gripper_close_ratio
   * - DefaultValue: 0.1
   */
  float m_gripper_close_ratio;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">

  //photointerrupta
  RTC::TimedFloatSeq m_in;
  /*!
   */
  InPort<RTC::TimedFloatSeq> m_inIn;


  //camera
  Img::TimedCameraImage m_camera;
  /*!
   */
  InPort<Img::TimedCameraImage> m_cameraIn;

  //gripper open or close
  RTC::TimedLongSeq m_gripperOC;
  /*!
   */
  OutPort<RTC::TimedLongSeq> m_gripperOCOut;
  
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_manipCommon_LPort;
  /*!
   */
  RTC::CorbaPort m_manipMiddle_LPort;
  /*!
   */
  RTC::CorbaPort m_manipCommon_RPort;
  /*!
   */
  RTC::CorbaPort m_manipMiddle_RPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM_LEFT::ManipulatorCommonInterface_Common> m_manipCommon_L;
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM_LEFT::ManipulatorCommonInterface_Middle> m_manipMiddle_L;
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Common> m_manipCommon_R;
  /*!
   */
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Middle> m_manipMiddle_R;

  bool State;
  int count;
  float Choucho;
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

         JARA_ARM::CarPosWithElbow targetPos;
         JARA_ARM::JointPos_var m_jointPos;
         JARA_ARM_LEFT::JointPos_var m_LjointPos;
	 int m_j0counter;
	 int m_j1counter;

	 coil::TimeValue m_sleepTime;

	 cv::Mat m_buffer;

	 std::ofstream m_JointLog;

	 std::string m_logDir;
};


extern "C"
{
  DLL_EXPORT void KnottingMotionGeneratorInit(RTC::Manager* manager);
};

#endif // ARMIMAGEGENERATOR_H
