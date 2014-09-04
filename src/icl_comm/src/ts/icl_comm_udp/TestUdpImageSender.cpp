// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Grischa Liebel
 * \date    2012-05-16
 *
 */
//----------------------------------------------------------------------
#include "TestUdpImageSender.h"

#include <icl_comm_udp/UdpImageSender.h>
#include <opencv2/highgui/highgui.hpp>

namespace icl_comm {
namespace udp {
CPPUNIT_TEST_SUITE_REGISTRATION ( TestUdpImageSender );

void TestUdpImageSender::TestSending()
{
  std::string ip = "127.0.0.1";
  icl_comm::udp::UdpImageSender* m_udp_sender = new icl_comm::udp::UdpImageSender(ip, 5555);
  CPPUNIT_ASSERT(!m_udp_sender->isSending()); // Initially this should return false

  //send standard color picture
  IplImage* test_image_color = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
  bool success = m_udp_sender->SendIplImage(test_image_color);
  CPPUNIT_ASSERT(success);
  CPPUNIT_ASSERT(m_udp_sender->isSending());
  cvReleaseImage(&test_image_color);

  //send standard grayscale picture
  IplImage* test_image_grayscale = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
  success = m_udp_sender->SendIplImage(test_image_grayscale);
  CPPUNIT_ASSERT(success);
  CPPUNIT_ASSERT(m_udp_sender->isSending());
  cvReleaseImage(&test_image_grayscale);

  //send picture with maximum dimensions
  IplImage* test_image_max = cvCreateImage(cvSize(2048, 2048), IPL_DEPTH_8U, 3);
  success = m_udp_sender->SendIplImage(test_image_max);
  CPPUNIT_ASSERT(success);
  CPPUNIT_ASSERT(m_udp_sender->isSending());
  cvReleaseImage(&test_image_max);

  //send picture with too great height
  IplImage* test_image_too_high = cvCreateImage(cvSize(640, 4096), IPL_DEPTH_8U, 1);
  success = m_udp_sender->SendIplImage(test_image_too_high);
  CPPUNIT_ASSERT(success == false);
  cvReleaseImage(&test_image_too_high);

  //send picture with too many bytes per line
  IplImage* test_image_too_big = cvCreateImage(cvSize(2048, 480), IPL_DEPTH_32F, 3);
  success = m_udp_sender->SendIplImage(test_image_too_big);
  CPPUNIT_ASSERT(success == false);
  cvReleaseImage(&test_image_too_big);
  delete m_udp_sender;
}

}
}
