#include <cppunit/CompilerOutputter.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/XmlOutputter.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>
#include <cppunit/extensions/TestFactoryRegistry.h>

#include "icl_comm_udp/Logging.h"

#include <string>
#include <fstream>

using namespace CPPUNIT_NS;
using namespace std;

int main(int argc, char* argv[])
{
  icl_core::logging::initialize(argc, argv);

  // Informiert Test-Listener ueber Testresultate
  TestResult testresult;

  // Listener zum Sammeln der Testergebnisse registrieren
  TestResultCollector collectedresults;
  testresult.addListener(&collectedresults);

  // Test-Suite ueber die Registry im Test-Runner einfuegen
  TestRunner testrunner;
  testrunner.addTest(TestFactoryRegistry::getRegistry().makeTest());
  testrunner.run(testresult);

  CompilerOutputter compileroutputter(&collectedresults, std::cerr);
  compileroutputter.write();

  icl_core::logging::tLoggingManager::instance().shutdown();

  // Rueckmeldung, ob Tests erfolgreich waren
  return collectedresults.wasSuccessful() ? 0 : 1;
}
