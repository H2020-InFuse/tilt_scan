#include <boost/test/unit_test.hpp>
#include <slam-tilt_scan/Dummy.hpp>

using namespace slam-tilt_scan;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    slam-tilt_scan::DummyClass dummy;
    dummy.welcome();
}
