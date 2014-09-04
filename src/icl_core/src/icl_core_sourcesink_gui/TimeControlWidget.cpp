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

#include "TimeControlWidget.h"

//! \todo use future version without QtGui/
#include <QtGui/QApplication>
#include <QtGui/QKeyEvent>
#include <QtCore/QTimer>

#include "ui_TimeControl.h"

namespace icl_core {
namespace sourcesink {

TimeControlWidget::TimeControlWidget()
  : m_playing(false),
    m_process_one_frame(false)
{
  Ui_TimeControl form;
  form.setupUi(this);
  connect(form.playpauseButton, SIGNAL(clicked(bool)), this, SLOT(setPlaying(bool)));
}

TimeControlWidget::~TimeControlWidget()
{

}

void TimeControlWidget::keyPressEvent(QKeyEvent *event)
{
//  std::cout << std::string("You Pressed: ") << event->text() << std::endl;

  int const multiplier = event->modifiers() & Qt::ControlModifier
      ? ( event->modifiers() & Qt::AltModifier ? 5 : 2 )
      : ( event->modifiers() & Qt::AltModifier ? 3 : 1 );

  switch (event->key())
  {
  case Qt::Key_Q: QCoreApplication::instance()->quit(); break;

  // Overlay and mode toggling
//  case Qt::Key_P: d->togglePlaybackControls();           break;

  // Playback controls
  case Qt::Key_Space: togglePlaying();                break;
  case Qt::Key_P: setPlaying(true);                   break;
  case Qt::Key_W: setPlaying(false);                  break;
  case Qt::Key_1: seekRelative(-1e0 * multiplier);    break;
  case Qt::Key_2: seekRelative(+1e0 * multiplier);    break;
  case Qt::Key_3: seekRelative(-1e1 * multiplier);    break;
  case Qt::Key_4: seekRelative(+1e1 * multiplier);    break;
  case Qt::Key_5: seekRelative(-1e2 * multiplier);    break;
  case Qt::Key_6: seekRelative(+1e2 * multiplier);    break;
  case Qt::Key_7: seekRelative(-1e3 * multiplier);    break;
  case Qt::Key_8: seekRelative(+1e3 * multiplier);    break;
  case Qt::Key_9: seekRelative(-1e4 * multiplier);    break;
  case Qt::Key_0: seekRelative(+1e4 * multiplier);    break;

  }

}

void TimeControlWidget::wheelEvent(QWheelEvent *event)
{
  // Wheel: 1, Ctrl+Wheel: 30, Alt+Wheel: 60, Ctrl+Alt+Wheel: 90
  int const multiplier = event->modifiers() & Qt::ControlModifier
      ? ( event->modifiers() & Qt::AltModifier ? 90 : 30 )
      : ( event->modifiers() & Qt::AltModifier ? 60 : 1 );
  int const progress = multiplier * (event->delta() > 0 ? 1 : -1);
  seekRelative(progress);
}

void TimeControlWidget::showEvent(QShowEvent *event)
{
  if (!event->spontaneous())
  {
    QTimer::singleShot(0, this, SLOT(processNextFrame()));
  }
}


void TimeControlWidget::processNextFrame()
{
  if (!m_source)
  {
    return;
  }

  if (isPlaying() || m_process_one_frame)
  {
    if (!process())
    {
      //! \todo handle process fail!
    }

    if (m_process_one_frame)
    {
      m_process_one_frame = false;
      return;
    }

    if (isPlaying())
    {
      // we need to seek the source one step if we are playing
      if(m_source->seekNext())
      {
        // reinitiate this function
        initiateProcessing();
      }
    }
  }
}

void TimeControlWidget::initiateProcessing()
{
  QTimer::singleShot(0, this, SLOT(processNextFrame()));
}

void TimeControlWidget::setSource(const icl_core::sourcesink::DataSourceBase::Ptr &source)
{
  if (!source->isSeekable())
  {
    //! \todo replace this by logging in future (after we moved this to sourcesink)
    std::cout << " ERROR: given source \"" + source->identifier() + "\" not seekable!" << std::endl;
//    throw std::runtime_error("given source \"" + source->identifier() + "\" not seekable!");
  }
  else
  {
    m_source = source;
  }
}

const icl_core::sourcesink::DataSourceBase::Ptr TimeControlWidget::source() const
{
  return m_source;
}

void TimeControlWidget::seekRelative(const int offset)
{
  if (!m_source)
  {
    return;
  }

  m_source->seekRelative(offset);

  if (!isPlaying())
  {
    m_process_one_frame = true;
    initiateProcessing();
  }
}

void TimeControlWidget::setPlaying(bool playing)
{
  if (playing != m_playing)
  {
    m_playing = playing;

    // initiate player loop
    if ( isPlaying() )
    {
      initiateProcessing();
    }
  }
}

bool TimeControlWidget::isPlaying() const
{
  return m_playing;
}

void TimeControlWidget::togglePlaying()
{
  setPlaying(!isPlaying());
}


}
}

#include "TimeControlWidget.moc"

