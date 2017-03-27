#include "indicator.h"
#include <QPalette>
#include <QLabel>
#include <QPainter>
#include <QFontMetrics>
#include <QApplication>

Indicator::Indicator(const QString &str, QWidget *parent)
	: QWidget(parent)	
    , m_color(/*QColor(Qt::lightGray)*/palette().color(QPalette::Window))
	, m_text(str)    
{	
	QFontMetrics fm(QApplication::font());
    setMinimumSize(fm.width(m_text) + 8, fm.height() + 8);
	update();
}

void Indicator::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    QPainter painter(this);
    painter.fillRect(rect(), m_color);
    QPen pen;
    pen.setColor(Qt::darkGray);
    painter.setPen(pen);
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    pen.setColor(Qt::black);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.drawText(rect(), Qt::AlignCenter, m_text);
}
void Indicator::setColor(const QColor &color)
{
	m_color = color;
	update();
}

