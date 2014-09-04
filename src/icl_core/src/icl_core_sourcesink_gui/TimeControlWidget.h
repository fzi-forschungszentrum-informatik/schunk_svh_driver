// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    2014-01-15
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_GUI_TIME_CONTROL_WIDGET_H
#define ICL_CORE_SOURCESINK_GUI_TIME_CONTROL_WIDGET_H

#include "ImportExport.h"

#include "icl_core_sourcesink/DataSource.h"

//! \todo use future version without QtGui/
//#include <QWidget>
#include <QtGui/QWidget>



namespace icl_core {
namespace sourcesink {

/**
 * @brief The TimeControlWidget allows play/pause and seeking on DataSourceBase
 * objects.
 *
 * Inherit from this to implement your own player. Example: icl_ibeoplayer_noapi
 *
 * \todo move this class to a more central library so that every library that
 * uses icl_core::sourcesink can also use this Widget
 *
 */
class ICL_CORE_SOURCESINK_GUI_IMPORT_EXPORT TimeControlWidget: public QWidget
{
  Q_OBJECT

public:

  TimeControlWidget();
  ~TimeControlWidget();


  /**
   * @brief setSource sets the Source that we want to control.
   * @param source The source - should be seekable.
   */
  void setSource(const DataSourceBase::Ptr& source);
  const DataSourceBase::Ptr source() const;


  /**
   * @brief The main processing function
   *
   * This function is triggered everytime a new frame is given.
   *
   * Inherit from TimeControlWidget and implement your processing in this function!
   *
   */
  virtual bool process() = 0;


protected Q_SLOTS:
  void processNextFrame();

  /**
   * @see QWidget#keyPressEvent
   */
  virtual void keyPressEvent(QKeyEvent *event);

  /**
   * @see QWidget#wheelEvent
   */
  void wheelEvent (QWheelEvent * event);


protected:
  void initiateProcessing();

  virtual void showEvent (QShowEvent * event);


private:
  void seekRelative(const int offset);

private Q_SLOTS:
  void setPlaying(bool playing);

private:
  bool isPlaying() const;

  void togglePlaying();

  DataSourceBase::Ptr m_source;

  bool m_playing;

  bool m_process_one_frame;
};


}
}

#endif // ICL_CORE_SOURCESINK_GUI_TIME_CONTROL_WIDGET_H
