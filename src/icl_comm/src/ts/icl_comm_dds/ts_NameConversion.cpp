// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jan Oberlaender
 * \date    2011-11-07
 *
 */
//----------------------------------------------------------------------
#include <icl_comm_dds/NameConversion.h>

#include <boost/test/unit_test.hpp>

using icl_comm::dds::convertMcaToDdsName;
using icl_comm::dds::convertDdsToMcaName;
using icl_core::String;

BOOST_AUTO_TEST_SUITE(ts_NameConversion)

struct NamePair
{
  /*! Provide an \a mcaname and the corresponding \a ddsname.  The
   *  back-conversion of the \a ddsname is expected to be identical to
   *  the original.
   */
  NamePair(const icl_core::String& mcaname, const icl_core::String& ddsname)
    : mcaname(mcaname), ddsname(ddsname), mcaname_back(mcaname)
  { }

  /*! Provide an \a mcaname and the corresponding \a ddsname.  The
   *  result of back-converting the \a ddsname is provided is \a
   *  mcaname_back.
   */
  NamePair(const icl_core::String& mcaname, const icl_core::String& ddsname, const icl_core::String& mcaname_back)
    : mcaname(mcaname), ddsname(ddsname), mcaname_back(mcaname_back)
  { }

  icl_core::String mcaname;
  icl_core::String ddsname;
  icl_core::String mcaname_back;
};

// A real example
NamePair t1_real_example("DC:segway1:/SegwayX2-HAL/Motion/RedundantOdometry (BackupSensors)/CurrentPose",
                         "DCX3Asegway1X3AX2FSegwayX582X2DHALX2FMotionX2FRedundantOdometryX20X28BackupSensorsX29X2FCurrentPose");

// Special case: illegal characters
NamePair t2_illegal_chars("M:Zone:/Group\x1f/\x7f",
                          "MX3AZoneX3AX2FGroupX3FX2FX3F",
                          "M:Zone:/Group?/?");
// Only illegal characters
NamePair t3_all_illegal("\x01\x02\x03\x04\x1d\x1e\x1f\x7f\xb0",
                        "X3FX3FX3FX3FX3FX3FX3FX3FX3F",
                        "?????????");
// Empty strings
NamePair t4_empty("",
                  "");
// Only one incorrect escape code
NamePair t5_one_escape("\x01",
                       "X3F",
                       "?");

// Back-conversion: illegal chars
NamePair tb1_illegal_chars("",
                           "M:Zone:/Group\x1f/\x7f",
                           "M?Zone??Group???");
// Back-conversion: invalid escape codes
NamePair tb2_invalid_escape("",
                            "MX3ZoneX3AX2FGroupX3FXVabX3",
                            "M?one:/Group??ab?");
// Back-conversion: invalid escape codes 2
NamePair tb3_invalid_escape("",
                            "MXXZoneX3AX2_FGroupX",
                            "M?Zone:?FGroup?");
// Back-conversion: single invalid character
NamePair tb4_one_char("",
                      "\\",
                      "?");
// Back-conversion: single invalid character 2
NamePair tb5_one_char("",
                      "X",
                      "?");

BOOST_AUTO_TEST_CASE(ConvertMcaToDds)
{
  BOOST_CHECK_EQUAL(convertMcaToDdsName(t1_real_example.mcaname),  t1_real_example.ddsname);
  BOOST_CHECK_EQUAL(convertMcaToDdsName(t2_illegal_chars.mcaname), t2_illegal_chars.ddsname);
  BOOST_CHECK_EQUAL(convertMcaToDdsName(t3_all_illegal.mcaname),   t3_all_illegal.ddsname);
  BOOST_CHECK_EQUAL(convertMcaToDdsName(t4_empty.mcaname),         t4_empty.ddsname);
  BOOST_CHECK_EQUAL(convertMcaToDdsName(t5_one_escape.mcaname),    t5_one_escape.ddsname);
}

BOOST_AUTO_TEST_CASE(ConvertDdsToMca)
{
  BOOST_CHECK_EQUAL(convertDdsToMcaName(t1_real_example.ddsname),    t1_real_example.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(t2_illegal_chars.ddsname),   t2_illegal_chars.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(t3_all_illegal.ddsname),     t3_all_illegal.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(t4_empty.ddsname),           t4_empty.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(t5_one_escape.ddsname),      t5_one_escape.mcaname_back);

  BOOST_CHECK_EQUAL(convertDdsToMcaName(tb1_illegal_chars.ddsname),  tb1_illegal_chars.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(tb2_invalid_escape.ddsname), tb2_invalid_escape.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(tb3_invalid_escape.ddsname), tb3_invalid_escape.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(tb4_one_char.ddsname),       tb4_one_char.mcaname_back);
  BOOST_CHECK_EQUAL(convertDdsToMcaName(tb5_one_char.ddsname),       tb5_one_char.mcaname_back);
}

BOOST_AUTO_TEST_SUITE_END()
