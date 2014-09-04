#ifndef ICL_COMM_UDP_TEST_UDPIMAGESENDER_H
#define ICL_COMM_UDP_TEST_UDPIMAGESENDER_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace icl_comm {
namespace udp {

class TestUdpImageSender : public CPPUNIT_NS :: TestFixture
{
    // Add all tests to suite
    CPPUNIT_TEST_SUITE ( TestUdpImageSender );
    CPPUNIT_TEST ( TestSending );
    CPPUNIT_TEST_SUITE_END ();

protected:
    void TestSending();
};

} // namespace udp
} // namespace icl_comm

#endif // ICL_COMM_UDP_TEST_UDPIMAGESENDER_H
